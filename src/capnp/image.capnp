@0xa5565e1abe1d96a9;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Image {

    header @0 :import "header.capnp".Header;

    pixelFormat @1 :Text;
    width @2 :UInt32;
    height @3 :UInt32;
    
    # exposure, gain

}