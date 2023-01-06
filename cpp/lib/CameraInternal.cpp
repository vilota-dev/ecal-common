#include "CameraInternal.hpp"

#include <iostream>

namespace vk 
{


void CameraInternal::init(const CameraParams &params) {
    m_params = params;

    m_messageSyncHandler.init(m_params.camera_topics.size());

    eCAL::Initialize(0, nullptr, m_params.ecal_process_name.c_str());
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I feel good !");

    std::cout << "subscribe to camera topics:" << std::endl;
    size_t idx = 0;
    for (auto& topic : m_params.camera_topics) {
        std::cout << " - " << topic << std::endl;
        auto callback = std::bind(&CameraInternal::cameraCallbackInternal, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, idx);
        m_imageSubMap.emplace(topic, topic);
        m_imageSubMap.at(topic).AddReceiveCallback(callback);

        idx++;
    }

    if (!m_params.imu_topic.empty()) {
        std::cout << "subscribe to imu topic:" << std::endl;
        std::cout << " - " << m_params.imu_topic << std::endl;
        auto callback = std::bind(&CameraInternal::imuCallbackInternal, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        m_imuSubMap.emplace(m_params.imu_topic, m_params.imu_topic);
        m_imuSubMap.at(m_params.imu_topic).AddReceiveCallback(callback);
    }
    
}

void CameraInternal::registerSyncedCameraCallback(callbackCamera callback) {

    std::cout << "adding synced camera callback " << m_registeredImageCallbacks.size() << std::endl;
    m_registeredImageCallbacks.push_back(callback);
}

void CameraInternal::registerImuCallback(callbackImu callback) {
    std::cout << "adding imu callback " << m_registeredImuCallbacks.size() << std::endl;
    m_registeredImuCallbacks.push_back(callback);
}

void CameraInternal::cameraCallbackInternal(const char* ecal_topic_name, ecal::Image::Reader ecal_msg, const long long ecal_ts, size_t idx) {
    

    // we need to have logic synchronizing all cameras

    if (m_params.camera_exact_sync) {

        CameraFrameData::Ptr msg = std::make_shared<CameraFrameData>();

        auto header = ecal_msg.getHeader();

        msg->ts = header.getStamp();
        msg->seq = header.getSeq();

        // std::cout << ecal_topic_name << " data received at ts = " << msg->ts << std::endl;

        m_messageSyncHandler.addMessage(idx, header.getStamp(), header.getSeq(), msg);

        auto synced = m_messageSyncHandler.tryGet();

        if (synced.size()) {
            std::cout << "synced image message at " << synced[0]->ts << std::endl;
        }

    }else
        throw std::runtime_error("not implemented non-sync version of camera callbacks");
}

void CameraInternal::imuCallbackInternal(const char* ecal_topic_name, ecal::Imu::Reader ecal_msg, const long long ecal_ts) {

    // std::cout << topic_name << " data received at ts = " << ts << std::endl;

}

CameraInternal::~CameraInternal()
{
    eCAL::Finalize();
}


} // namespace vk