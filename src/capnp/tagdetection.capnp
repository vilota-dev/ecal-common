@0x83675942c5c5cbd1;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("vkc");

# Tag Detection shall encode image as well for debugging purpose, and could be switched off when desired
# This is due to the fact that detection may run at a lower frequency

enum TagFamily {
    tag16h5 @0;
    tag25h7 @1;
    tag25h9 @2;
    tag36h10 @3;
    tag36h11 @4;
    tag16h5Border2 @5;
    tag25h7Border2 @6;
    tag25h9Border2 @7;
    tag36h10Border2 @8;
    tag36h11Border2 @9;
}

struct AprilGrid {
    startId @0 :UInt32;
    increment @1 :Int32; # can be negative
    family @2 :TagFamily;
    tagCols @3 :UInt8;
    tagRows @4 :UInt8;
    tagSpacing @5 :Float32;
    tagSize @6 :Float32;
    gridId @7 :UInt64; # generated unique hash
}

# Tag could possibly belong to a known grid; grid will have size info
struct TagDetection {
    id @0 :UInt32;
    family @1 :TagFamily;
    hammingDistance @2 :UInt8;
    # corners
    pointsPolygon @3 :List(Float32); # 0~1 should be stored, instead of pixel value
    gridId @4 :UInt64; # should be a hash, zero means no grid associated
}

struct TagDetections {
    header @0 :import "header.capnp".Header; # aligned to image stamp, sequence has its own count
    cameraExtrinsic @1 :import "sensorextrinsic.capnp".SensorExtrinsic;
    image @2 :import "image.capnp".Image; # image data may not be populated, but intrinsic would
    tags @3 :List(TagDetection);
    grids @4 :List(AprilGrid);
}