@0xa5565e1abe1d96a9;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("vkc");

struct MotionMeta {
    instantaneousAngularVelocity @0 :import "vector3.capnp".Vector3d; # timestamp at middle of exposure
    averageAngularVelocity @1 :import "vector3.capnp".Vector3d; # average from mid-exposre of previous frame to the current
    shockLikelihood @2 :Float32; # from previous frame to current frame
    deltaUs @3 :Float32; # mid-exposure from previous frame
}

struct Image {

    enum Encoding {
        mono8 @0;
        mono16 @1;
        yuv420 @2;
        bgr8 @3;
        jpeg @4;
        png @5;
        h264 @6;
        h265 @7;
    }

    header @0 :import "header.capnp".Header;

    encoding @1 :Encoding;
    width @2 :UInt32;
    height @3 :UInt32;
    step @4 :UInt32; # bytes in a row
    data @5 :Data;
    
    # metadata
    exposureUSec @6 :UInt32;
    gain @7 :UInt32;

    sensorIdx @8 :Int8; # physical camera index
    streamName @9 :Text;

    intrinsic @10 :import "cameraintrinsic.capnp".CameraIntrinsic;
    extrinsic @11 :import "sensorextrinsic.capnp".SensorExtrinsic;

    mipMapLevels @12 :UInt8; # 0 means original image, and so on
    mipMapBrightness @13 :UInt8; # calculated from the smallest available mipmap
    mipMapBrightnessChange @14 :Float32; # change of brightness against rolling average, offset by exp and gain changes

    motionMeta @15 :MotionMeta;
}