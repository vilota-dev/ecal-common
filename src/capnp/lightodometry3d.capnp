@0x951de96fde3ac0b3;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct LightOdometry3d {

    stamp @0 :UInt64;

    # position
    px @1 :Float32;
    py @2 :Float32;
    pz @3 :Float32;

    # orientation
    qw @4 :Float32;
    qx @5 :Float32;
    qy @6 :Float32;
    qz @7 :Float32;

}