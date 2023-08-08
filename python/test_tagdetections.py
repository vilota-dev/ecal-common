#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

capnp.add_import_hook(['../src/capnp'])

import tagdetection_capnp as eCALTags

imshow_map = {}

def callback(type, topic_name, msg, ts):

    # need to remove the .decode() function within the Python API of ecal.core.subscriber StringSubscriber
    with eCALTags.TagDetections.from_bytes(msg) as tagsMsg:
        imageMsg = tagsMsg.image
        tags = tagsMsg.tags

        if (imageMsg.encoding == "mono8"):
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height, imageMsg.width, 1))
            imshow_map[topic_name + " mono8"] = mat
        elif (imageMsg.encoding == "yuv420"):
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height * 3 // 2, imageMsg.width, 1))
            mat = cv2.cvtColor(mat, cv2.COLOR_YUV2BGR_IYUV)
            imshow_map[topic_name + " yuv420"] = mat
        elif (imageMsg.encoding == "bgr8"):
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height, imageMsg.width, 3))
            imshow_map[topic_name + " bgr8"] = mat
        elif (imageMsg.encoding == "jpeg"):
            mat_jpeg = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = cv2.imdecode(mat_jpeg, cv2.IMREAD_COLOR)
            imshow_map[topic_name + " jpeg"] = mat
        else:
            raise RuntimeError("unknown encoding: " + imageMsg.encoding)
        
        print (f"topic: {topic_name}, num tags = {len(tagsMsg.tags)}")
        for tag in tags:
            tag_pose = tag.poseInCameraFrame
            position = tag_pose.position
            print(f"\tid = {tag.id}, x = {position.x}, y = {position.y}, z = {position.z}")
        print()


def main():
    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_tagdetections_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    # create subscriber and connect callback

    n = len(sys.argv)
    if n == 1:
        topic = "S0/tags/camd"
    elif n == 2:
        topic = sys.argv[1]
    else:
        raise RuntimeError("Need to pass in exactly one parameter for topic")

    print(f"Streaming topic {topic}")
    sub = CapnpSubscriber("TagDetections", topic)
    sub.set_callback(callback)
    
    # idle main thread
    while ecal_core.ok():
        for im in imshow_map:
            cv2.imshow(im, imshow_map[im])
        cv2.waitKey(10)
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()
