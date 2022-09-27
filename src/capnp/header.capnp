@0xc61974417b74b4cc;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

struct Header {

    enum ClockDomain {
        monotonic @0;
        realtime @1;
    }

    seq @0 :UInt64;
    stamp @1 :UInt64;
    frameId @2 :Text;
    clockDomain @3 :ClockDomain;
}