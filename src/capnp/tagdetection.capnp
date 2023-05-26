@0x83675942c5c5cbd1;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

# Tag Detection shall encode image as well for debugging purpose, and could be switched off when desired
# This is due to the fact that detection may run at a lower frequency

struct FourPoints {
    pt1 @0 :import "vector2.capnp".Vector2f; # bottom-left
    pt2 @1 :import "vector2.capnp".Vector2f; # bottom-right
    pt3 @2 :import "vector2.capnp".Vector2f; # top-right
    pt4 @3 :import "vector2.capnp".Vector2f; # top-left
}

# the size of TagDetection is known
struct TagDetection {
    id @0 :UInt32;
    hammingDistance @1 :UInt8;
    tagSize @2 :Float32;
    areaPixel @3 :Float32; # number of pixels the tag occupies in the image

    tagCenter @4 :import "vector2.capnp".Vector2f;

    # corners
    pointsPolygon @5 :FourPoints;
    pointsUndistortedPolygon @6 :FourPoints;

    poseInCameraFrame @7 :import "se3.capnp".Se3; # in RDF camera frame

}

struct TagDetections {
    enum TagFamily {
        Tag16h5 @1;
        Tag25h9 @2;
        Tag36h10 @3;
        Tag36h11 @4;
        Tag36h9 @5;
        TagCircle21h7 @6;
        TagCircle49h12 @7;
        TagCustom48h12 @8;
        TagStandard41h12 @9;
        TagStandard52h13 @10;
    }
    header @0 :import "header.capnp".Header; # aligned to image stamp, sequence has its own count

    cameraExtrinsic @1 :import "sensorextrinsic.capnp".SensorExtrinsic;

    tagFamily @2 :TagFamily;

    image @3 :import "image.capnp".Image; # image data may not be populated

    tags @4 :List(TagDetection);
}