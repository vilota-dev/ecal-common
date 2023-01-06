#include "CameraInternal.hpp"

#include <iostream>

namespace vk 
{


void CameraInternal::init(const CameraParams &params) {
    m_params = params;

    std::cout << "subscribe to camera topics:" << std::endl;
    for (auto& topic : m_params.camera_topics) {
        std::cout << " - " << topic << std::endl;
    }

    std::cout << "subscribe to imu topic:" << std::endl;
    std::cout << m_params.imu_topic << std::endl;

    eCAL::Initialize(0, nullptr, m_params.ecal_process_name.c_str());

    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I feel good !");

}

void CameraInternal::registerSyncedCameraCallback(callbackCamera callback) {

}

void CameraInternal::registerImuCallback(callbackImu callback) {

}

CameraInternal::~CameraInternal()
{
    eCAL::Finalize();
}


} // namespace vk