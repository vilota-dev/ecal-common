@0xd081a777c9aa537c;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("vkc");

struct So3 {
    x @0 :Float64;
    y @1 :Float64;
    z @2 :Float64;
    w @3 :Float64; 
}