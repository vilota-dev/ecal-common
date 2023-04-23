@0x83675942c5c5cbd1;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

# Tag Detection shall encode image as well for debugging purpose, and could be switched off when desired
# This is due to the fact that detection may run at a lower frequency

struct TagDetection {
    id @0 :UInt32;
    hammingDistance @1 :UInt8;
    tagSize @2 :Float32;
    areaPixel @3 :Float32; # number of pixels the tag occupies in the image

    tagCenter @4 :import "vector2.capnp".Vector2f;

    # corners
    pointsPolygon @5 :List(import "vector2.capnp".Vector2f);
    pointsUndistortedPolygon @6 :List(import "vector2.capnp".Vector2f);

    poseInCameraFrame @7 :import "se3.capnp".Se3; # in RDF camera frame

}

struct TagDetections {
    header @0 :import "header.capnp".Header; # aligned to image stamp, sequence has its own count

    image @1 :import "image.capnp".Image; # image data may not be populated

    tags @2 :List(TagDetection);
}