@0xd5992f4464b8cfaf;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Se3 {
    position @0 :import "vector3d.capnp".Vector3d;
    orientation @1 :import "so3.capnp".So3;
}