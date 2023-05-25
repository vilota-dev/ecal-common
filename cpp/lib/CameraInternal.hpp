#pragma once

#include <map>
#include <queue>
#include <mutex>

#include <ecal_camera/CameraInterface.hpp>

#include <ecal/ecal.h>
#include <ecal/msg/capnproto/helper.h>
#include <ecal/msg/capnproto/subscriber.h>
#include <ecal/msg/capnproto/publisher.h>


#include <capnp/image.capnp.h>
#include <capnp/imu.capnp.h>

#include <capnp/cameracontrol.capnp.h>

namespace vk 
{

class CameraInternal : public CameraInterface {

  public:
    void init(const CameraParams &params);

    void registerSyncedCameraCallback(callbackCamera callback);
    void registerImuCallback(callbackImu callback);

    void sendCameraControl(const CameraControlData& data);
    
    // void sendJsonIn(const std::string& topic, const std::string& content);

    ~CameraInternal();

  private:
    void run();

    void cameraCallbackInternal(const char* topic_name, ecal::Image::Reader msg, const long long ts, size_t idx);
    void imuCallbackInternal(const char* topic_name, ecal::Imu::Reader msg, const long long ts);

    std::map<std::string, eCAL::capnproto::CSubscriber<ecal::Image>> m_imageSubMap;
    std::map<std::string, eCAL::capnproto::CSubscriber<ecal::Imu>> m_imuSubMap;

    // regarding lastSeq
    std::map<size_t, std::uint64_t> m_lastSeqCameraFrameMap;
    std::uint64_t m_lastSeqImu = 0;

    // regarding calibration
    std::map<size_t, CameraCalibration> m_cameraCalibrationMap;
    ImuCalibration m_imuCalibration;

    std::vector<callbackCamera> m_registeredImageCallbacks;
    std::vector<callbackImu> m_registeredImuCallbacks;

    MessageSynchroniserExact<CameraFrameData::Ptr> m_messageSyncHandler;

    // camera control
    std::shared_ptr<eCAL::capnproto::CPublisher<ecal::CameraControl>> m_cameraControlPub;

    bool m_ecalInitialisedOutside;

};

} // namespace vk

