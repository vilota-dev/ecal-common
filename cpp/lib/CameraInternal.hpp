#pragma once

#include <ecal_camera/CameraInterface.hpp>

namespace vk 
{

class CameraInternal : public CameraInterface {

        void init(const CameraParams &params);

        void registerSyncedCameraCallback(callbackCamera callback);
        void registerImuCallback(callbackImu callback);

};

} // namespace vk

