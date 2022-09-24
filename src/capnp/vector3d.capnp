@0xb582b480fc4d3622;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Vector3d {
    x @0 :Float64;
    y @1 :Float64;
    z @2 :Float64; 
}