#include <ecal_camera/CameraFactory.hpp>

#include <thread>
#include <chrono>

#include <iostream>

void callbackSyncedCameras(const std::vector<vk::CameraFrameData::Ptr>& dataVector) {

    std::uint64_t seq = dataVector[0]->seq;

    std::cout << "synced camera received, seq = " << seq << std::endl;
}

void callbackImu(const vk::ImuFrameData::Ptr data) {

}

int main() {

    auto camera = vk::CameraFactory::getCameraHandler();

    vk::CameraParams param;

    param.camera_topics = {"S0/camd", "S0/camc", "S0/camb"};
    param.imu_topic = {"S0/imu"};

    camera->init(param);

    camera->registerSyncedCameraCallback(callbackSyncedCameras);
    camera->registerImuCallback(callbackImu);

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}