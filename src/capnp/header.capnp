@0xc61974417b74b4cc;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("vkc");

struct Header {

    # Enum is deprecated; no longer in use.
    enum ClockDomain {
        monotonic @0;
        realtime @1;
    }

    seq @0 :UInt64;
    stampMonotonic @1 :UInt64;      # Monotonic time when data is created (captured, measured etc).
    frameId @2 :Text;               # Not intended to be used, only for compatibility of ros message header.
    clockDomain @3 :ClockDomain;    # Deprecated; had always been "monotonic" variant.
    latencyDevice @4 :UInt64;       # Latency introduced until reception at device CPU.
    clockOffset @5 : Int64;         # Offset to be added to stampMonotonic to obtain system time.
}