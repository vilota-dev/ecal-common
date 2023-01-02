@0xebb5b4dfd4cfc717;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Imu {
    header @0 :import "header.capnp".Header;

    linearAcceleration @1 :import "vector3d.capnp".Vector3d;
    angularVelocity @2 :import "vector3d.capnp".Vector3d;

    streamName @3 :Text;
}