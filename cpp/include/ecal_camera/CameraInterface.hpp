#pragma once

#include <functional>
#include <deque>
#include <memory>
#include <iostream>

#include <vector>
#include <map>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
// #include <opencv2/opencv.hpp>

namespace vk 
{

enum SynchroniserReturnMode {
    CALLBACK_MODE,
    POLLING_MODE
};

template <typename T, SynchroniserReturnMode M = POLLING_MODE>
class MessageSynchroniserExact {

  public:
    using CallbackF = std::function<void(std::vector<T>)>;

    void init(size_t N, const std::vector<std::string> &names = {},
              std::string prefix = {}, size_t queueSize = 10, bool realtime = true) {
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
        m_queueSize = queueSize;

        m_realtime = realtime;
    }

    void addMessage(size_t idx, std::uint64_t ts, std::uint64_t seq, T msg) {
        if (m_lastTsMap[idx] > 0) {
            if (m_lastTsMap[idx] > ts)
                std::cout << "warn: ts regression detected, from "
                          << m_lastTsMap[idx] << " to " << ts
                          << std::endl;
        }

        {
            std::lock_guard<std::mutex> lock(m_mutexQueue);
            m_queueMap[idx].push_back(std::make_pair(ts, msg));

            if (m_queueMap[idx].size() > m_queueSize) {

                std::string name;
                if (m_names.size())
                    name = m_names[idx];

                // it is ok to have failed the queue before first sync get done
                if (m_queueMap[idx].size() % m_queueSize == 1) {

                    if ( m_lastSyncTs > 0 ) {
                        std::cout << name << ": too much message in the queue, sync msg is broken?" << std::endl;
                    } else {
                        std::cout << name << ": buffer full while no sync detected yet..." << std::endl;
                    }

                    for (size_t i = 0; i < m_N; i++) {
                        int size = m_queueMap[i].size();
                        std::string name = m_names.size() ? m_names[i] : std::to_string(i);
                        std::cout << " - " << name << ": " << size << std::endl;
                    }
                }

                m_queueMap[idx].pop_front();
            }
        }

        m_lastTsMap[idx] = ts;
        m_lastSeqMap[idx] = seq;

        {
            std::lock_guard<std::mutex> lock(m_mutexQueue);

            if (m_realtime) {
                auto ret = tryGet(true);
                std::vector<T> prevRet;
                while (ret.size()) {
                    prevRet = ret;
                    ret = tryGet(true);
                }
                // insert ret to front of queue
                if (prevRet.size()) {
                    for (size_t i = 0; i < m_N; i++) {
                        m_queueMap[i].push_front(std::make_pair(m_lastSyncTs, prevRet[i]));
                    }
                }
            } else {
                cleanupQueues();
            }

            if (M == CALLBACK_MODE) {
                auto ret = tryGet(true);
                if (ret.size()) {
                    std::for_each(m_callbacks.begin(), m_callbacks.end(), [&](auto& cb) {
                        cb(ret);
                    });
                }
            }
        }
    }

    template <SynchroniserReturnMode U = M>
    std::enable_if_t<U == POLLING_MODE, std::vector<T>> tryGet() {
        return tryGet(false);
    }

    template <SynchroniserReturnMode U = M>
    std::enable_if_t<U == CALLBACK_MODE, void> registerCallback(CallbackF cb) {
        m_callbacks.push_back(cb);
    }

  private:
    // queueLocked must be true only internally
    std::vector<T> tryGet(bool queueLocked) {
        auto lock = !queueLocked
                        ? std::unique_lock<std::mutex>(m_mutexQueue)
                        : std::unique_lock<std::mutex>();

        std::vector<T> ret;
        for (auto& queue : m_queueMap) {
            if (!queue.size())
                return ret; // some queues are empty
        }
        uint64_t syncTs = cleanupQueues();

        if (syncTs) {
            // sanity check
            if (m_lastSyncTs > syncTs) {
                std::cout << "error: synced message ts regression from " << m_lastSyncTs
                          << " to " << syncTs << ", skipping" << std::endl;
                ret.clear();
                return ret; // empty return
            }

            // sync found, returning and popping
            ret.resize(m_N);
            for (size_t i = 0; i < m_N; i++) {
                auto val = m_queueMap[i].front().second;
                ret[i] = val;
                m_queueMap[i].pop_front();
            }

            m_lastSyncTs = syncTs; // equals to maxTs
            return ret;
        }

        return ret; // empty return
    }

    // returns 0 if sync not found
    // ensure m_mutexQueue is locked
    uint64_t cleanupQueues() {
        for (auto& queue : m_queueMap) {
            if (!queue.size())
                return 0; // some queues are empty
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
                    m_queueMap[i].pop_front();

                    // if the queue is not empty, try again
                    if (m_queueMap[i].size())
                        i--;
                    continue;
                }
            }
            return 0;
        }

        return minTs;
    }

    size_t m_N;
    std::vector<std::string> m_names;
    std::vector<std::deque<std::pair<std::uint64_t,T>>> m_queueMap;
    std::mutex m_mutexQueue;
    std::vector<std::uint64_t> m_lastTsMap;
    std::vector<std::uint64_t> m_lastSeqMap;
    std::uint64_t m_lastSyncTs;

    size_t m_queueSize;
    bool m_realtime;
    std::vector<CallbackF> m_callbacks{};
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

    std::string product_name;
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
