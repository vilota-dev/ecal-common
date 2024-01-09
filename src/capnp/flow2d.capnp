@0x9d805819b50be7fe;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("ecal");

# Scalar r2 = p3dt[0] * p3dt[0] + p3dt[1] * p3dt[1];
# Scalar r = std::sqrt(r2);
# polar_azimuthal_angle[i][0] = std::atan2(r, p3dt[2]);
# polar_azimuthal_angle[i][1] = std::atan2(p3dt[0], p3dt[1]);

struct Flow2d {
    id @0 :UInt64; # the upper 32-bit encodes the hash of the camera/stereo, the lower 32-bit encodes the actual id
    position @1 :import "vector2.capnp".Vector2d; # should be in 0~1 range, respect to width and height
    radial @2 :Float32; # radial in radian, from the positive z-axis
    azimuth @3 :Float32; # azimuth in radian, from the positive y-axix, counter clockwise

    level @4 :UInt8; # mipmap level
    age @5 :UInt32;
}

struct HFOpticalFlowResult {

    header @0 :import "header.capnp".Header;
    cameraTopic @1 :Text;

    meanFlow @2 :List(Float32); # a ratio to the mean of height and width of the image, negative number means N.A.
    flowDensity @3 :List(Float32); # 9 numbers, centre, then clockwise from 0'o clock. sum indicate if we have at least 27 active tracking (3 average)
    flowData @4 :List(Flow2d); 
    
}