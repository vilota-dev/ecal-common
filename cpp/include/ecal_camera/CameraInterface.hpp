#pragma once

#include <functional>
#include <queue>
#include <memory>
#include <iostream>

#include <vector>
#include <map>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
// #include <opencv2/opencv.hpp>

namespace vk 
{

template <typename T>
class MessageSynchroniserExact {

  public:
    void init(size_t N, const std::vector<std::string> &names = {}, std::string prefix = {}) {
        m_N = N;
        if (names.size()) {
            assert(names.size() == N);
            m_names = names;
        }

        for (auto& name : m_names)
            name = prefix + name;
            
        m_queueMap.resize(m_N);
        m_lastTsMap.resize(m_N);
        m_lastSeqMap.resize(m_N);

        m_lastSyncTs = 0;
    }

    void addMessage(size_t idx, std::uint64_t ts, std::uint64_t seq, T msg) {
        if (m_lastTsMap[idx] > 0) {
            if (m_lastTsMap[idx] > ts)
            std::cout << "warn: ts regression detected, from " << m_lastTsMap[idx] << " to " << ts << std::endl;
        }
        // else{
        //     std::cout << "first message received at synchroniser for " << m_names.size() ? m_names[idx] : idx << std::endl;
        // }

        {
            std::lock_guard<std::mutex> lock(m_mutexQueue);
            m_queueMap[idx].push(std::make_pair(ts, msg));
            
            constexpr size_t MAX_BUFFER_SIZE = 50;
            if (m_queueMap[idx].size() > MAX_BUFFER_SIZE) {

                std::string name;
                if (m_names.size())
                    name = m_names[idx];

                // it is ok to have failed the queue before first sync get done
                if (m_queueMap[idx].size() % MAX_BUFFER_SIZE == 1) {
                    
                    if ( m_lastSyncTs > 0 ) {
                        std::cout << name << ": too much message in the queue, sync msg is broken?" << std::endl;
                    }else{
                        std::cout << name << ": buffer full while no sync detected yet..." << std::endl;
                    }

                    for (size_t i = 0; i < m_N; i++) {
                        int size = m_queueMap[i].size();
                        std::string name = m_names.size() ? m_names[i] : std::to_string(i);
                        std::cout << " - " << name << ": " << size << std::endl;
                    }
                }


                
                    
            }
            
        }

        m_lastTsMap[idx] = ts;
        m_lastSeqMap[idx] = seq;
    }

    std::vector<T> tryGet() {

        std::lock_guard<std::mutex> lock(m_mutexQueue);

        std::vector<T> ret;

        for (auto& queue : m_queueMap) {
            if (!queue.size())
                return ret; // some queues are empty
        }

        std::uint64_t minTs, maxTs;
        minTs = maxTs = m_queueMap[0].front().first;

        // calc min and max ts
        for (size_t i = 1; i < m_N; i++) {
            auto ts = m_queueMap[i].front().first;
            if (minTs > ts)
                minTs = ts;
            if (maxTs < ts)
                maxTs = ts;
        }

        if (minTs != maxTs) {
            // clean up all items smaller than maxTs
            for (size_t i = 0; i < m_N; i++) {
                auto ts = m_queueMap[i].front().first;
                if (ts < maxTs) {
                    m_queueMap[i].pop();

                    // if the queue is not empty, try again
                    if (m_queueMap[i].size())
                        i--;
                    continue;
                }
            }
            return ret;

        }else{
            // sync found, returning and popping
            ret.resize(m_N);
            for (size_t i = 0; i < m_N; i++) {
                ret[i] = m_queueMap[i].front().second;
                m_queueMap[i].pop();
            }

            // sanity check
            if (m_lastSyncTs >= minTs) {
                std::cout << "error: synced message ts regression from " << m_lastSyncTs << " to " << minTs << ", skipping" << std::endl;
                ret.clear();
                return ret; // empty return
            }

            m_lastSyncTs = minTs; // equals to maxTs
            return ret;
        }
    }

  private:

    size_t m_N;
    std::vector<std::string> m_names;
    std::vector<std::queue<std::pair<std::uint64_t,T>>> m_queueMap;
    std::mutex m_mutexQueue;
    std::vector<std::uint64_t> m_lastTsMap;
    std::vector<std::uint64_t> m_lastSeqMap;
    std::uint64_t m_lastSyncTs;

};

struct CameraParams {
    bool half_resolution = false;
    std::string tf_prefix = "S0/";
    std::vector<std::string> camera_topics = {"camb", "camc", "camd"}; // the order would be preserved, but should not be assumed
    std::string imu_topic = {"imu"};
    std::string camera_control_topic = {"camera_control_in"};
    bool camera_exact_sync = true;
    std::string ecal_process_name = "camimu interface cpp";

    std::vector<std::string> idxTopicMap; // being populated, without prefix
    std::map<std::string, uint16_t> topicIdxMap; // being populated, without prefix
};

struct CameraCalibration {
    std::map<std::string, Eigen::VectorXd> intrinsicMap; // there might be multiple calibration for the same camera
    std::uint64_t lastModifiedIntrinsic = 0;
    bool rectified;

    Eigen::Isometry3d imu_T_cam;
    Eigen::Isometry3d body_T_cam;
    std::uint64_t lastModifiedExtrinsic = 0;
};

struct CameraFrameData {

    typedef std::shared_ptr<CameraFrameData> Ptr;

    std::uint16_t idx; // index showed up in callbackCamera callback
    std::string prefixed_topic; // e.g. "S0/cama"
    std::string topic; // topic without prefix, e.g. "cama"

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
    
    Eigen::Vector3d gyroNoiseStd;
    Eigen::Vector3d accelNoiseStd;
    Eigen::Vector3d gyroBiasStd;
    Eigen::Vector3d accelBiasStd;

    int updateRate;
    int timeOffsetNs;
    std::uint64_t lastModifiedIntrinsic = 0;

    Eigen::Isometry3d body_T_imu;
    std::uint64_t lastModifiedExtrinsic = 0;
};

struct ImuFrameData {

    typedef std::shared_ptr<ImuFrameData> Ptr;

    std::string prefixed_topic;

    std::uint64_t ts;
    std::uint64_t seq;
    std::uint64_t lastSeq = 0;
    int seqIncrement;

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

        // virtual void sendJsonIn(const std::string& topic, const std::string& content) = 0;

        CameraParams getParams() {return m_params;}

    protected:

        CameraParams m_params;

};

} // namespace vk
