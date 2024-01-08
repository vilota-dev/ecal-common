#!/usr/bin/env python3

import sys
import time

# capnp
import capnp
from utils import SyncedImageSubscriber
import ecal.core.core as ecal_core

capnp.add_import_hook(['../src/capnp'])

import odometry3d_capnp as eCALOdometry3d
import tagdetection_capnp as eCALTagDetections

# symforce deps
import symforce
symforce.set_epsilon_to_symbol()

from symforce import typing as T
import symforce.symbolic as sf
from symforce.values import Values
from symforce.opt.optimizer import Optimizer
from symforce.opt.factor import Factor

# rerun viz
import cv2
import numpy as np
import rerun as rr

# global
optimized = False

min_pose_count = 10
min_tag_count = 1

images = []

poses = []
in_between_Ts = []
# list of { tag id -> idx in all_tag_corners } (NOT VALUE)
pose_tag_corners = []
# [ corners v2 ]
all_tag_corners = []
# { tag id -> idx in all_tags } (NOT VALUE)
tags = {}
# [ corners v3 ]
all_tags = []
intrinsics = None
diagonal_sigmas = sf.V6(0.05, 0.05, 0.05, 0.1, 0.1, 0.1)

nwu_T_edn = sf.Pose3(
    sf.Rot3(q=sf.Quaternion(sf.V3(0.5, -0.5, 0.5), -0.5)), 
    sf.V3(0, 0, 0))

# viz utils 
def image_msg_to_cvmat(imageMsg):
    if (imageMsg.encoding == "mono8"):

        mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
        mat = mat.reshape((imageMsg.height, imageMsg.width, 1))

        # cv2.imshow("mono8", mat)
        # cv2.waitKey(3)
    elif (imageMsg.encoding == "yuv420"):
        mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
        mat = mat.reshape((imageMsg.height * 3 // 2, imageMsg.width, 1))

        mat = cv2.cvtColor(mat, cv2.COLOR_YUV2BGR_IYUV)

    elif (imageMsg.encoding == "bgr8"):
        mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
        mat = mat.reshape((imageMsg.height, imageMsg.width, 3))
    elif (imageMsg.encoding == "jpeg"):
        mat_jpeg = np.frombuffer(imageMsg.data, dtype=np.uint8)
        mat = cv2.imdecode(mat_jpeg, cv2.IMREAD_COLOR)
    else:
        raise RuntimeError("unknown encoding: " + imageMsg.encoding)
    
    return mat

# symforce stuff
def build_inital_values():
    return Values(
        poses = poses,
        in_between_Ts = in_between_Ts,
        all_tags = all_tags,
        all_tag_corners = all_tag_corners,
        diagonal_sigmas = diagonal_sigmas,
        epsilon=sf.numeric_epsilon
    )

def project_to_image(point: sf.V3, odom_T_body: sf.Pose3, epsilon: sf.Scalar):
    odom_T_body_edn = odom_T_body * nwu_T_edn
    point_body = odom_T_body_edn.inverse() * point

    x, y, z = point_body[0], point_body[1], point_body[2]
    fx, fy, cx, cy, k1, k2, k3, k4 = intrinsics

    r2 = x * x + y * y
    r = sf.sqrt(r2)
    theta = sf.atan2(r, z, epsilon=epsilon)
    theta2 = theta * theta

    r_theta = k4 * theta2
    r_theta += k3
    r_theta *= theta2
    r_theta += k2
    r_theta *= theta2
    r_theta += k1
    r_theta *= theta2
    r_theta += 1
    r_theta *= theta

    mx = x * r_theta / sf.Max(r, epsilon)
    my = y * r_theta / sf.Max(r, epsilon)

    proj_u = fx * mx + cx
    proj_v = fy * my + cy

    return sf.V2(proj_u, proj_v)

# kb4 intrinsincs: (fx, fy, cx, cy, k1, k2, k3, k4)
def kb4_reprojection_residual(point: sf.V3, odom_T_body: sf.Pose3,
                          obs_pt: sf.V2, epsilon: sf.Scalar):
    return project_to_image(point, odom_T_body, epsilon) - obs_pt

def odometry_residual(
    odom_T_a: sf.Pose3,
    odom_T_b: sf.Pose3,
    a_T_b: sf.Pose3,
    diagonal_sigmas: sf.V6,
    epsilon: sf.Scalar
):
    a_T_b_predicted = odom_T_a.inverse() * odom_T_b
    tangent_error = a_T_b_predicted.local_coordinates(a_T_b, epsilon=epsilon)
    return T.cast(sf.V6, sf.M.diag(diagonal_sigmas.to_flat_list()).inv() * sf.V6(tangent_error))

def planar_tag_residual(point1: sf.V3, point2: sf.V3, 
                        point3: sf.V3, point4: sf.V3, epsilon: sf.Scalar):
    p1p2 = point2 - point1
    p1p3 = point3 - point1
    p1p4 = point4 - point1

    p1p2 = p1p2 / p1p2.norm(epsilon=epsilon)
    p1p3 = p1p3 / p1p3.norm(epsilon=epsilon)
    p1p4 = p1p4 / p1p4.norm(epsilon=epsilon)

    normal = p1p2.cross(p1p3)
    return sf.V1(normal.dot(p1p4))

def tag_distances_residual(point1: sf.V3, point2: sf.V3,
                            point3: sf.V3, point4: sf.V3, epsilon: sf.Scalar):
    p1p2 = point2 - point1
    p2p3 = point3 - point2
    p3p4 = point4 - point3
    p4p1 = point1 - point4

    a_norm = p1p2.norm(epsilon=epsilon)
    b_norm = p2p3.norm(epsilon=epsilon)
    c_norm = p3p4.norm(epsilon=epsilon)
    d_norm = p4p1.norm(epsilon=epsilon)

    return sf.V4(a_norm - b_norm, b_norm - c_norm, c_norm - d_norm, d_norm - a_norm)

def build_factors():
    for i in range(len(poses) - 1):
        yield Factor(
            residual=odometry_residual,
            keys=[f"poses[{i}]", f"poses[{i+1}]", f"in_between_Ts[{i}]", "diagonal_sigmas", "epsilon"],
        )
    
    for i in range(len(poses)):
        for tag_id in pose_tag_corners[i].keys():
            all_tag_corners_idx = pose_tag_corners[i][tag_id]
            all_tags_idx = tags[tag_id]
            for j in range(4):
                yield Factor(
                    residual=kb4_reprojection_residual,
                    keys=[f"all_tags[{all_tags_idx + j}]", f"poses[{i}]", f"all_tag_corners[{all_tag_corners_idx + j}]", "epsilon"],
                )

    # not particularly useful
    # for i in range(0, len(all_tags), 4):
    #     yield Factor(
    #         residual=planar_tag_residual,
    #         keys=[f"all_tags[{i}]", f"all_tags[{i+1}]", f"all_tags[{i+2}]", f"all_tags[{i+3}]", "epsilon"],
    #     )
    #     yield Factor(
    #         residual=tag_distances_residual,
    #         keys=[f"all_tags[{i}]", f"all_tags[{i+1}]", f"all_tags[{i+2}]", f"all_tags[{i+3}]", "epsilon"],
    #     )

def viz(optimizer: Optimizer, result: Optimizer.Result):
    rr.init("vk_viewer_rr")
    rr.spawn(memory_limit='10%')
    # rr.serve()
    rr.set_time_seconds("host_monotonic_time", time.monotonic_ns())

    rr.log("S0", rr.ViewCoordinates.FLU, timeless=True)
    points = []

    # draw grid with a single connected lines
    min_x = -50
    max_x = 50
    min_y = -50
    max_y = 50
    step = 5

    even_line = True
    for x in range(min_x, max_x + step, step):
        if even_line:
            line = [[x, max_y, 0], [x, min_y, 0]]
        else:
            line = [[x, min_y, 0], [x, max_y, 0]]
        even_line = not even_line
        points += line

    for y in range(min_y, max_y + step, step):
        if even_line:
            line = [[min_x, y, 0], [max_x, y, 0]]
        else:
            line = [[max_x, y, 0], [min_x, y, 0]]
        even_line = not even_line
        points += line

    rr.log("S0/grid", rr.LineStrips3D(points, radii=[0.01]))
    
    iter_idx = 0

    mats = []
    for image in images:
        mats.append((image.header.stamp, image_msg_to_cvmat(image)))

    values_per_iter = [optimizer.load_iteration_values(stats.values) for stats in result.iterations]

    for values in values_per_iter:
        poses = values["poses"]
        all_tags = values["all_tags"]
        
        odom_points = []

        for (msg_idx, (ts, mat)) in enumerate(mats):
            rr.set_time_nanos("host_monotonic_time", ts)
            rr.log(f"S0/tag_image_iter_{iter_idx}", rr.DisconnectedSpace())

            odom_T_body = poses[msg_idx]
            tmp_odom = sf.Pose3.symbolic("tmp_odom")

            pos = odom_T_body.position()
            rot = odom_T_body.rotation().data
            qx, qy, qz, qw = rot[0], rot[1], rot[2], rot[3]
            x, y, z, = pos[0], pos[1], pos[2]

            odom_T_body = sf.Pose3(sf.Rot3(sf.Quaternion(sf.V3(qx, qy, qz), qw)), sf.V3(x, y, z))

            mat = mat.copy() #idontcare
            for point in all_tags:
                point = sf.V3(point[0], point[1], point[2])
                uv = project_to_image(point, odom_T_body, sf.numeric_epsilon)

                # insert circles for uv points in mat
                cv2.circle(mat, (int(uv[0]), int(uv[1])), 2, (0, 0, 255), 2)

            odom_points.append(odom_T_body.position())

            # we have to ways to prevent all images drawn to the same screen
            rr.log(f"S0/tag_image_iter_{iter_idx}", rr.Image(mat))

        rr.log(f"S0/odom_points_{iter_idx}", rr.DisconnectedSpace())
        rr.log(f"S0/odom_points_{iter_idx}", rr.LineStrips3D(odom_points, class_ids=[iter_idx], radii=[0.1]))

        iter_idx += 1

def optimize():
    # Create a problem setup and initial guess
    initial_values = build_inital_values()

    # Create factors
    factors = build_factors()

    # Select the keys to optimize - the rest will be held constant
    optimized_keys = [f"poses[{i}]" for i in range(1, len(poses))] + \
                     [f"all_tags[{i}]" for i in range(len(all_tags))]

    # Create the optimizer
    optimizer = Optimizer(
        factors=factors,
        optimized_keys=optimized_keys,
        debug_stats=True,  # Return problem stats for every iteration
        params=Optimizer.Params(verbose=True),  # Customize optimizer behavior
    )

    # Solve and return the result
    result = optimizer.optimize(initial_values)
    viz(optimizer, result)
    
    # Print values
    print(f"Num iterations: {len(result.iterations) - 1}")
    print(f"Final error: {result.error():.6f}")
    print(f"Status: {result.status}")
    for i, pose in enumerate(result.optimized_values["poses"]):
        print(f"Pose {i}: t = {pose.position()}, heading = {pose.rotation()}")
    for i, tag in enumerate(result.optimized_values["all_tags"]):
        print(f"Tag {i}: {tag}")

# read data
def callback(msg, _):
    global optimized, poses, in_between_Ts, pose_tag_corners, all_tag_corners, tags, all_tags, intrinsics

    if optimized:
        return
    
    print(len(poses), len(tags))

    if len(poses) >= min_pose_count and len(tags) >= min_tag_count:
        optimized = True
        print("Optimizing...")
        optimize()
        print("Done")
        return
    
    t_temp = msg["S0/tags/camd"]
    tags_msg = t_temp.tags
    odom = msg["S0/vio_odom"].pose

    # TODO: gravity align to 2d
    t = odom.position
    q = odom.orientation

    rot = sf.Rot3(q=sf.Quaternion(sf.V3(q.x, q.y, q.z), q.w))
    pos = sf.V3(t.x, t.y, t.z)
    odom_pose = sf.Pose3(rot, pos)

    print(odom_pose)

    # image intrinsics
    if intrinsics is None:
        kb4 = t_temp.image.intrinsic.kb4
        pinhole = kb4.pinhole
        intrinsics = (pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy, kb4.k1, kb4.k2, kb4.k3, kb4.k4)

    images.append(t_temp.image)

    # legacy tag detection msg
    tag_pairs = {}
    for tag in tags_msg:
        p = tag.poseInCameraFrame.position
        tq = tag.poseInCameraFrame.orientation

        odom_T_tag = odom_pose * nwu_T_edn * sf.Pose3(
            sf.Rot3(q=sf.Quaternion(sf.V3(tq.x, 
                                          tq.y, 
                                          tq.z), tq.w)), 
            sf.V3(p.x, p.y, p.z))

        tx, ty, tz = odom_T_tag.position().x , odom_T_tag.position().y, odom_T_tag.position().z
        print(f"Tag {tag.id}: {tx}, {ty}, {tz}")

        pts = tag.pointsPolygon
        corners = []

        for point in [pts.pt1, pts.pt2, pts.pt3, pts.pt4]:
            corners.append(sf.V2(point.x, point.y))

        tag_pairs[tag.id] = len(all_tag_corners)
        all_tag_corners += corners

        if tag.id not in tags:
            tags[tag.id] = len(all_tags)

            all_tags += [sf.V3(0,0,0)] * 4
            # [sf.V3(tx, ty, tz)] * 4
            # [sf.V3(0,0,0)] * 4

    if len(poses) > 0:
        prev_pose = poses[-1]
        in_between_Ts.append(prev_pose.inverse() * odom_pose)

    pose_tag_corners.append(tag_pairs)
    poses.append(odom_pose)

    print("------------------")

def main():
    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_odometry_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    types = ["TagDetections", "Odometry3d"]
    topics = ["S0/tags/camd", "S0/vio_odom"]
    classes = [eCALTagDetections.TagDetections, eCALOdometry3d.Odometry3d]

    sub = SyncedImageSubscriber(types, topics, classes)
    sub.register_callback(callback)
    
    # idle main thread
    while ecal_core.ok():
        time.sleep(0.1)
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()