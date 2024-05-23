@0xd569317e0d67b155;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("vkc");

# https://www.ros.org/reps/rep-0103.html where NWU is defined for body frame
# https://www.ros.org/reps/rep-0105.html where ENU is defined for global frame

struct VioPose {
    stamp @0 :UInt64;
    pose @1 :import "se3.capnp".Se3;
    kf @2 :Bool;
    group @3 :Int32;
}

struct VioLandmark {
    id @0 :UInt64; # keypoint id
    hostedKfStamp @1 :UInt64; # hosted keyframe stamp
    observedStamps @2 :List(UInt64); # stamp that has observation of the landmark
    invDist @3 :Float32; # inverse distance calculated
    point3d @4 :import "vector3.capnp".Vector3f; # position of landmark in odom frame (NWU)
}

struct VioState {
    header @0 :import "header.capnp".Header;
    states @1 :List(VioPose);
    frames @2 :List(VioPose);

    landmarks @3 :List(VioLandmark);

    biasGyro @4 :import "vector3.capnp".Vector3f;
    biasAccel @5 :import "vector3.capnp".Vector3f;
}


struct Odometry3d {

    enum BodyFrame {
        nwu @0;
        ned @1;
    }

    enum ReferenceFrame {
        enu @0; # typically with global reference, map frame of ROS
        nwu @1; # typically with local reference, odom frame of ROS
        ned @2; # typically with MAV
    }

    enum VelocityFrame {
        none @0; # not implemented
        body @1; # body frame velocity
        reference @2; # refernece frame velocity
    }

    header @0 :import "header.capnp".Header;
    bodyFrame @1 :BodyFrame;
    referenceFrame @2 :ReferenceFrame;
    velocityFrame @3 :VelocityFrame;

    pose @4 :import "se3.capnp".Se3;
    poseCovariance @5 :List(Float32); # either 21 or 36 numbers of float

    twist @6 :import "twist3.capnp".Twist3;
    twistCovariance @7 :List(Float32); # either 21 or 36 numbers of float

    resetCounter @8 :UInt32;

    metricVisionFailureLikelihood @9 :Float32; # 0 means vision functioning, 1 means imu only
    metricInertialFailureLikelihood @10 :Float32; # 0 means imu function, 1 means imu bias estimation is bad
    estimatedFailureModeDrift @11 :Float32; # additional drift due to vision failure
    metricFailureVio @12 :Int8; # single metric to determine if the VIO is in bad state: -1 is unknown, 0 is normal and 1 is failure

}