@0x85b66463488de4eb;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Disparity {
    enum Encoding {
        disparity8 @0; # encoded as integer disparity
        disparity16 @1; # encoded as scaled disparity of scaling of 8
        depth16 @2; # encoded as uint16, in millimeter
        depth32 @3; # encoded as float32, in meter
    }

    header @0 :import "header.capnp".Header;

    encoding @1 :Encoding;
    width @2 :UInt32;
    height @3 :UInt32;
    step @4 :UInt32;
    data @5 :Data; # row major array of bytes with indicated encoding

    fx @6 :Float32;
    fy @7 :Float32;
    cx @8 :Float32;
    cy @9 :Float32;
    baseline @10 :Float32;

    streamName @11 :Text; # typically are "_rect" version to indicated rectified stream

    pinholeRotation @12 :import "so3.capnp".So3;

    maxDisparity @13 :UInt32;

}