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

find_package(CapnProto REQUIRED)
find_package(eCAL REQUIRED)

include_directories(
  ../include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/../../src)


add_library(ecal_camera 
  CameraInternal.cpp
  CameraFactory.cpp
  ${CAPNP_SRCS}
  ${CAPNP_HDRS})
target_link_libraries(ecal_camera 
  PUBLIC ${OpenCV_LIBS}
  PRIVATE Threads::Threads CapnProto::capnp)

# alias will make public dependencies go through
add_library(ecal_camera::ecal_camera ALIAS ecal_camera)