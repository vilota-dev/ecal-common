#pragma once

#include <functional>
#include <memory>

#include <vector>
#include <map>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
// #include <opencv2/opencv.hpp>

namespace vk 
{

struct CameraParams {
    std::string tf_prefix = "S0/";
    std::vector<std::string> camera_topics = {"camb", "camc", "camd"}; // the order would be preserved, but should not be assumed
    std::string imu_topic = {"imu"};
    std::string camera_control_topic = {"camera_control_in"};
    bool camera_exact_sync = true;
    std::string ecal_process_name = "camimu interface cpp";

    std::vector<std::string> idxTopicMap; // being populated
    std::map<std::string, uint16_t> topicIdxMap; // being populated
};

struct CameraCalibration {
    std::map<std::string, Eigen::VectorXd> intrinsicMap; // there might be multiple calibration for the same camera
    std::uint64_t lastModifiedIntrinsic;
    bool rectified;

    Eigen::Isometry3d imu_T_cam;
    Eigen::Isometry3d body_T_cam;
    std::uint64_t lastModifiedExtrinsic = 0;
};

struct CameraFrameData {

    typedef std::shared_ptr<CameraFrameData> Ptr;

    std::string prefixed_topic;

    std::uint64_t ts;
    std::uint64_t seq;
    std::uint64_t lastSeq = 0;

    std::string encoding;
    cv::Mat image;

    std::uint32_t exposureUSec;
    std::uint32_t gain;

    std::int8_t sensorIdx;

    CameraCalibration calib;

};

struct ImuCalibration {
    Eigen::Isometry3d body_T_imu;
    std::uint64_t lastModifiedExtrinsic = 0;
};

struct ImuFrameData {

    typedef std::shared_ptr<ImuFrameData> Ptr;

    std::string prefixed_topic;

    std::uint64_t ts;
    std::uint64_t seq;
    std::uint64_t lastSeq = 0;

    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;

    ImuCalibration calib;
};

struct CameraControlData {
    std::int8_t streaming = -1;

    std::int32_t exposureUSec = -1;
    std::int32_t gain = -1;

    std::int8_t exposureCompensation = -99;
    std::int8_t sensorIdx = -1;
};

class CameraInterface {

    public:
        typedef std::shared_ptr<CameraInterface> Ptr;

        typedef std::function<void(const std::vector<CameraFrameData::Ptr>&)> callbackCamera;
        typedef std::function<void(const ImuFrameData::Ptr)> callbackImu;

        virtual void init(const CameraParams &params) = 0;

        virtual void registerSyncedCameraCallback(callbackCamera callback) = 0;
        virtual void registerImuCallback(callbackImu callback) = 0;

        virtual void sendCameraControl(const CameraControlData& data) = 0;

        virtual void sendJsonIn(const std::string& topic, const std::string& content) = 0;

        CameraParams getParams() {return m_params;}

    protected:

        CameraParams m_params;

};

} // namespace vk