#pragma once

#include <ecal_camera/CameraInterface.hpp>

#include <ecal/ecal.h>
#include <ecal/msg/capnproto/helper.h>
#include <ecal/msg/capnproto/subscriber.h>


#include <capnp/image.capnp.h>
#include <capnp/imu.capnp.h>

namespace vk 
{

class CameraInternal : public CameraInterface {

  public:
    void init(const CameraParams &params);

    void registerSyncedCameraCallback(callbackCamera callback);
    void registerImuCallback(callbackImu callback);

    ~CameraInternal();

  private:
    void run();

    std::vector<eCAL::capnproto::CSubscriber<ecal::Image>> m_imageMap;

};

} // namespace vk

