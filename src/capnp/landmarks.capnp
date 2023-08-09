@0x81f4d3b7184ec1cf;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

# the size of TagDetection is known
struct Landmark {
    id @0 :UInt32;    
    size @1 :Float32;
    pose @2 :import "se3.capnp".Se3;
    covariance @3 :import "vector3.capnp".Vector3d; # TODO make more general
}

struct Landmarks {
    enum BodyFrame {
        nwu @0;
        ned @1;
    }

    header @0 :import "header.capnp".Header; # aligned to image stamp, sequence has its own count
    bodyFrame @1 :BodyFrame;
    landmarks @2 :List(Landmark);
}
