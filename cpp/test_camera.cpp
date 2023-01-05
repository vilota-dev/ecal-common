#include <ecal_camera/CameraFactory.hpp>

#include <thread>
#include <chrono>

int main() {

    auto camera = vk::CameraFactory::getCameraHandler();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}