#pragma once

#include <functional>
#include <memory>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace vk 
{

class CameraParams {

};

class CameraFrameData {

    std::uint64_t ts;
    std::uint64_t seq;
    std::uint64_t lastSeq;

    cv::Mat image;

    Eigen::Isometry3d imu_T_cam;
    Eigen::Isometry3d body_T_cam;

};

class ImuFrameData {
    std::uint64_t ts;
    std::uint64_t seq;
    std::uint64_t lastSeq;

    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;

    Eigen::Isometry3d body_T_imu;
};

class CameraInterface {

    public:
        typedef std::shared_ptr<CameraInterface> Ptr;

        typedef std::function<void(const std::vector<CameraFrameData>&)> callbackCamera;
        typedef std::function<void(const ImuFrameData)> callbackImu;

        virtual void init(const CameraParams &params) = 0;

        virtual void registerSyncedCameraCallback(callbackCamera callback) = 0;
        virtual void registerImuCallback(callbackImu callback) = 0;

};

} // namespace vk