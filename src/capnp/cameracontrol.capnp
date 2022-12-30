@0x8b94c342930b4a59;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct CameraControl {
    header @0 :import "header.capnp".Header;

    exposureUSec @1 :UInt32; # range 1 - 33000
    gain @2 :UInt32; # range 100 - 1600
    sensorName @3 :Text;
}