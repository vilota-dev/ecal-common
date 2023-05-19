# cmake_minimum_required(VERSION 3.10)
# project(cpp_api)

# enable C++17 standard (current directory scope)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# https://stackoverflow.com/questions/5395309/how-do-i-force-cmake-to-include-pthread-option-during-compilation
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# configure build types and build c-flags
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE
      Debug
      CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()

find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)

find_package(Eigen3 REQUIRED)

if (NOT TARGET CapnProto::capnp)
  find_package(CapnProto REQUIRED)
endif()

find_package(eCAL REQUIRED)

include_directories(
  ../include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src)


  set(CAPNPC_SRC_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS 
      ../../src/capnp/header.capnp
      ../../src/capnp/vector3.capnp
      ../../src/capnp/imu.capnp
      ../../src/capnp/imulist.capnp
      ../../src/capnp/image.capnp
      ../../src/capnp/so3.capnp
      ../../src/capnp/se3.capnp
      ../../src/capnp/cameraintrinsic.capnp
      ../../src/capnp/sensorextrinsic.capnp
      ../../src/capnp/cameracontrol.capnp
      )

add_library(ecal_camera
  CameraInternal.cpp
  CameraFactory.cpp
  ${CAPNP_SRCS}
  ${CAPNP_HDRS})
target_include_directories(ecal_camera
  INTERFACE ../include)
target_link_libraries(ecal_camera 
  PUBLIC ${OpenCV_LIBS} eCAL::core
  PRIVATE Threads::Threads CapnProto::capnp)

# alias will make public dependencies go through
add_library(ecal_camera::ecal_camera ALIAS ecal_camera)