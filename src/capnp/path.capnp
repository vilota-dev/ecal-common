@0x84e8b7f035de0a30;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Path {
    path @0 :List(import "lightodometry3d.capnp".LightOdometry3d);
}