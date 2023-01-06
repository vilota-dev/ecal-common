#pragma once

#include <functional>
#include <memory>

#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace vk 
{

struct CameraParams {
    std::vector<std::string> camera_topics;
    std::string imu_topic;
    bool camera_exact_sync = true;
    std::string ecal_process_name = "camimu interface cpp";
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

    protected:

        CameraParams m_params;

};

} // namespace vk