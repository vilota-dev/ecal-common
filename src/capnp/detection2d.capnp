@0x94839fd8bdb7c208;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Detection2d {

    labelIdx @0 :UInt32;
    labelString @1 :Text;
    xmin @2 :Float32;
    xmax @3 :Float32;
    ymin @4 :Float32;
    ymax @5 :Float32;
    confidence @6 :Float32;

}

struct Detections2d {

    header @0 :import "header.capnp".Header;

    detections @1 :List(Detection2d);

    width @2 :UInt32;
    height @3 :UInt32;

    imageData @4 :Data; # should be RGB
}