#include <ecal_camera/CameraFactory.hpp>

#include <thread>
#include <chrono>

#include <iostream>

void callbackSyncedCameras(const std::vector<vk::CameraFrameData::Ptr>& dataVector) {

    std::uint64_t seq = dataVector[0]->seq;

    const int N = dataVector.size();

    std::cout << "synced camera received, seq = " << seq << std::endl;

    std::uint64_t nowTns = std::chrono::steady_clock::now().time_since_epoch().count();
    if (dataVector[0]->seq % 10 == 0){
        for (int i = 0; i < N; i++){
            std::cout << "camera "<< i << " latency at test_camera = " 
                << (nowTns - dataVector[i]->ts) / 1e6 << " ms";
        }        
    }

}

void callbackImu(const vk::ImuFrameData::Ptr data) {

}

int main() {

    auto camera = vk::CameraFactory::getCameraHandler();

    vk::CameraParams param;

    param.tf_prefix = "S0/";
    param.camera_topics = {"camd", "camc", "camb"};
    param.imu_topic = {"imu"};

    camera->init(param);

    camera->registerSyncedCameraCallback(callbackSyncedCameras);
    camera->registerImuCallback(callbackImu);

    vk::CameraControlData control;
    camera->sendCameraControl(control);

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}