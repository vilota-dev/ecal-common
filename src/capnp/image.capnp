@0xa5565e1abe1d96a9;

struct Header {
    seq @0 :UInt64;
    stamp @1 :UInt64;
    frameId @2 :Text;
}

struct Image {

    header @0 :Header;

    pixelFormat @1 :Text;
    width @2 :UInt32;
    height @3 :UInt32;
    
    # exposure, gain

}