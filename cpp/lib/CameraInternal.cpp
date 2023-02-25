#include "CameraInternal.hpp"

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp> // for debugging

namespace vk 
{


void CameraInternal::init(const CameraParams &params) {
    m_params = params;

    if (params.half_resolution)
        std::cout << "half resolution mode enabled" << std::endl;

    m_messageSyncHandler.init(m_params.camera_topics.size(), m_params.camera_topics, m_params.tf_prefix);

    if (!eCAL::IsInitialized(0)) {
        eCAL::Initialize(0, nullptr, m_params.ecal_process_name.c_str());
        eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I feel good !");

        m_ecalInitialisedOutside = false;
    }else {
        std::cout << "eCAL already initialised" << std::endl;
        m_ecalInitialisedOutside = true;
    }
    

    std::cout << "subscribe to camera topics:" << std::endl;
    size_t idx = 0;
    for (auto& topic : m_params.camera_topics) {
        std::string prefixed_topic = m_params.tf_prefix + topic;
        std::cout << " - " << prefixed_topic << std::endl;
        auto callback = std::bind(&CameraInternal::cameraCallbackInternal, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, idx);
        m_imageSubMap.emplace(topic, prefixed_topic);
        m_imageSubMap.at(topic).AddReceiveCallback(callback);
        m_params.idxTopicMap.push_back(topic);
        m_params.topicIdxMap[topic] = idx;

        idx++;
    }

    if (!m_params.imu_topic.empty()) {
        std::string prefixed_topic = m_params.tf_prefix + m_params.imu_topic;
        std::cout << "subscribe to imu topic:" << std::endl;
        std::cout << " - " << prefixed_topic << std::endl;
        auto callback = std::bind(&CameraInternal::imuCallbackInternal, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        m_imuSubMap.emplace(m_params.imu_topic, prefixed_topic);
        m_imuSubMap.at(m_params.imu_topic).AddReceiveCallback(callback);
    }

    // initialise publishers

    if (!m_params.camera_control_topic.empty()) {
        std::string prefixed_topic = m_params.tf_prefix + m_params.camera_control_topic;
        m_cameraControlPub = std::make_shared<eCAL::capnproto::CPublisher<ecal::CameraControl>>();
        m_cameraControlPub->Create(prefixed_topic);
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

        const auto& header = ecal_msg.getHeader();
        const auto& streamName = ecal_msg.getStreamName().cStr();

        msg->idx = idx;
        msg->prefixed_topic = ecal_topic_name;
        msg->topic = m_params.idxTopicMap[idx];
        msg->ts = header.getStamp();
        msg->seq = header.getSeq();

        // logic on lastSeq
        if (m_lastSeqCameraFrameMap.count(idx)) {

            msg->lastSeq = m_lastSeqCameraFrameMap.at(idx);
            m_lastSeqCameraFrameMap.at(idx) = msg->seq;

            // sanity check for sequence
            if (msg->lastSeq == msg->seq) {
                std::cout << "error: duplicated image message " << ecal_topic_name << " detected: seq = " << msg->lastSeq << ", skipping " << std::endl;
                return;
            }else if (msg->lastSeq > msg->seq) {
                std::cout << "error: image message " << ecal_topic_name << " sequence regression, from " << msg->lastSeq << " to " << msg->seq << ", skipping " << std::endl;
                return;
            }

        }else {
            // first time obtaining the message
            msg->lastSeq = 0;
            m_lastSeqCameraFrameMap[idx] = msg->seq;
            std::cout << "first camera stream received for " << streamName << std::endl;
        }

        // logic on calibration
        const auto& intrinsicMsg = ecal_msg.getIntrinsic();
        const auto& extrinsicMsg = ecal_msg.getExtrinsic();

        bool updateIntrinsicCalibration = false;
        bool updateExtrinsicCalibration = false;

        if (m_cameraCalibrationMap.count(idx)) {

            auto& calibStored = m_cameraCalibrationMap.at(idx);

            if (intrinsicMsg.getLastModified() != calibStored.lastModifiedIntrinsic)
                updateIntrinsicCalibration = true;


            if (extrinsicMsg.getLastModified() != calibStored.lastModifiedExtrinsic)
                updateExtrinsicCalibration = true;

        }else {
            // first time setup for calibration

            m_cameraCalibrationMap[idx] = {};
            
            updateIntrinsicCalibration = true;
            updateExtrinsicCalibration = true;
        }

        if (updateExtrinsicCalibration || updateExtrinsicCalibration) {
            auto& calibStored = m_cameraCalibrationMap.at(idx);
            calibStored.rectified = ecal_msg.getIntrinsic().getRectified();
        }

        if (updateIntrinsicCalibration) {
            auto& calibStored = m_cameraCalibrationMap.at(idx);

            calibStored.lastModifiedIntrinsic = intrinsicMsg.getLastModified();
            std::cout << "received updated intrinsic for " << streamName << ", ts = " << calibStored.lastModifiedIntrinsic << std::endl;

            if (intrinsicMsg.hasPinhole()) {
                const auto& pinhole = intrinsicMsg.getPinhole();

                Eigen::Vector4d intrinsic = {pinhole.getFx(), pinhole.getFy(), pinhole.getCx(), pinhole.getCy()};

                if (m_params.half_resolution) {
                    intrinsic[0] /= 2;
                    intrinsic[1] /= 2;
                    intrinsic[2] /= 2;
                    intrinsic[3] /= 2;
                }

                calibStored.intrinsicMap["pinhole"] = intrinsic;
            }

            if (intrinsicMsg.hasKb4()) {
                const auto& kb4 = intrinsicMsg.getKb4();
                const auto& pinhole = kb4.getPinhole();

                Eigen::Matrix<double, 8, 1> intrinsic;
                intrinsic << pinhole.getFx(), pinhole.getFy(), pinhole.getCx(), pinhole.getCy(),
                    kb4.getK1(), kb4.getK2(), kb4.getK3(), kb4.getK4();

                if (m_params.half_resolution) {
                    intrinsic[0] /= 2;
                    intrinsic[1] /= 2;
                    intrinsic[2] /= 2;
                    intrinsic[3] /= 2;
                }

                calibStored.intrinsicMap["kb4"] = intrinsic;
            }

            if (intrinsicMsg.hasDs()) {
                const auto& ds = intrinsicMsg.getDs();
                const auto& pinhole = ds.getPinhole();

                Eigen::Matrix<double, 6, 1> intrinsic;
                intrinsic << pinhole.getFx(), pinhole.getFy(), pinhole.getCx(), pinhole.getCy(),
                    ds.getXi(), ds.getAlpha();

                if (m_params.half_resolution) {
                    intrinsic[0] /= 2;
                    intrinsic[1] /= 2;
                    intrinsic[2] /= 2;
                    intrinsic[3] /= 2;
                }

                calibStored.intrinsicMap["ds"] = intrinsic;
            }

            for (auto& item : calibStored.intrinsicMap) {
                std::cout << item.first << ": " << item.second.transpose() << std::endl;
            }

        }

        if (updateExtrinsicCalibration) {
            auto& calibStored = m_cameraCalibrationMap.at(idx);

            calibStored.lastModifiedExtrinsic = extrinsicMsg.getLastModified();
            std::cout << "received updated extrinsic for " << streamName << ", ts = " << calibStored.lastModifiedExtrinsic << std::endl;

            // body frame
            {
                Eigen::Vector3d position = {
                extrinsicMsg.getBodyFrame().getPosition().getX(),
                extrinsicMsg.getBodyFrame().getPosition().getY(),
                extrinsicMsg.getBodyFrame().getPosition().getZ()
                };

                Eigen::Quaterniond orientation = {
                    extrinsicMsg.getBodyFrame().getOrientation().getW(),
                    extrinsicMsg.getBodyFrame().getOrientation().getX(),
                    extrinsicMsg.getBodyFrame().getOrientation().getY(),
                    extrinsicMsg.getBodyFrame().getOrientation().getZ()
                };

                orientation.normalize();

                calibStored.body_T_cam.linear() = orientation.toRotationMatrix();
                calibStored.body_T_cam.translation() = position;

                std::cout << "body_T_cam: " << std::endl << calibStored.body_T_cam.affine() << std::endl;

            }

            // imu frame
            {
                Eigen::Vector3d position = {
                extrinsicMsg.getImuFrame().getPosition().getX(),
                extrinsicMsg.getImuFrame().getPosition().getY(),
                extrinsicMsg.getImuFrame().getPosition().getZ()
                };

                Eigen::Quaterniond orientation = {
                    extrinsicMsg.getImuFrame().getOrientation().getW(),
                    extrinsicMsg.getImuFrame().getOrientation().getX(),
                    extrinsicMsg.getImuFrame().getOrientation().getY(),
                    extrinsicMsg.getImuFrame().getOrientation().getZ()
                };

                orientation.normalize();

                calibStored.imu_T_cam.linear() = orientation.toRotationMatrix();
                calibStored.imu_T_cam.translation() = position;

                std::cout << "imu_T_cam: " << std::endl << calibStored.imu_T_cam.affine() << std::endl;

            }
            
        }

        msg->calib = m_cameraCalibrationMap.at(idx);

        if (ecal_msg.getEncoding() == ecal::Image::Encoding::MONO8) {
            msg->encoding = "mono8";

            // ecal_msg.getData().asBytes().begin()
            const cv::Mat rawImg(ecal_msg.getHeight(), ecal_msg.getWidth(), CV_8UC1, 
                const_cast<unsigned char*>(ecal_msg.getData().asBytes().begin()));
            
            if (m_params.half_resolution) {
                cv::resize(rawImg, msg->image, cv::Size(), 0.5, 0.5);
            }else
                msg->image = rawImg.clone();
        }else if (ecal_msg.getEncoding() == ecal::Image::Encoding::YUV420) {
            msg->encoding = "bgr8";
            const cv::Mat rawImg(ecal_msg.getHeight() * 3 / 2, ecal_msg.getWidth(), CV_8UC1, 
                const_cast<unsigned char*>(ecal_msg.getData().asBytes().begin()));
            
            if (m_params.half_resolution) {
                cv::Mat midImg;
                cv::resize(rawImg, midImg, cv::Size(), 0.5, 0.5);
                cv::cvtColor(midImg, msg->image, cv::COLOR_YUV2BGR_IYUV);
            }else
                cv::cvtColor(rawImg, msg->image, cv::COLOR_YUV2BGR_IYUV);
        }else{
            throw std::runtime_error("not implemented encoding");
        }

        if (!msg->image.isContinuous())
            throw std::runtime_error("cv mat is not continuous.");

        //exposure, gain and sensorIdx
        msg->exposureUSec = ecal_msg.getExposureUSec();
        msg->gain = ecal_msg.getGain();
        msg->sensorIdx = ecal_msg.getSensorIdx();

        m_messageSyncHandler.addMessage(idx, header.getStamp(), header.getSeq(), msg);

        // no more access to msg pointer beyond this point

        auto synced = m_messageSyncHandler.tryGet();

        if (synced.size()) {
            // std::cout << "synced image message at " << synced[0]->ts << std::endl;

            // DEBUG 
            // for (size_t i = 0; i < synced.size(); i++) {
            //     cv::imshow(m_idxMap[i], synced[i]->image);
            // }

            // cv::waitKey(3);

            for (auto& callback : m_registeredImageCallbacks) {
                callback(synced);
            }
            
        }

    }else
        throw std::runtime_error("not implemented non-sync version of camera callbacks");
}

void CameraInternal::imuCallbackInternal(const char* ecal_topic_name, ecal::Imu::Reader ecal_msg, const long long ecal_ts) {

    // std::cout << topic_name << " data received at ts = " << ts << std::endl;

    const auto& header = ecal_msg.getHeader();

    ImuFrameData::Ptr imuMessage = std::make_shared<ImuFrameData>();

    imuMessage->seq = header.getSeq();

    if (m_lastSeqImu == 0) {
        // first time receiving message
        imuMessage->lastSeq = 0;
        std::cout << "first time imu stream received" << std::endl;
    }else{
        imuMessage->lastSeq = m_lastSeqImu;

        // sanity check on sequence
        if (imuMessage->lastSeq == imuMessage->seq) {
            std::cout << "error: duplicated imu message " << ecal_topic_name  << " detected: seq = " << imuMessage->lastSeq << ", skipping " << std::endl;
            return;
        }else if (imuMessage->lastSeq > imuMessage->seq) {
            std::cout << "error: imu message " << ecal_topic_name << " sequence regression, from " << imuMessage->lastSeq << " to " << imuMessage->seq << ", skipping " << std::endl;
            return;
        }
    }

    m_lastSeqImu = imuMessage->seq;

    imuMessage->prefixed_topic = ecal_topic_name;
    imuMessage->ts = header.getStamp();

    imuMessage->gyro = {
        ecal_msg.getAngularVelocity().getX(),
        ecal_msg.getAngularVelocity().getY(),
        ecal_msg.getAngularVelocity().getZ()
    };

    imuMessage->accel = {
        ecal_msg.getLinearAcceleration().getX(),
        ecal_msg.getLinearAcceleration().getY(),
        ecal_msg.getLinearAcceleration().getZ()
    };

    // calibration logic
    bool updateIntrinsicCalibration = false;
    bool updateExtrinsicCalibration = false;

    if (imuMessage->calib.lastModifiedIntrinsic != ecal_msg.getIntrinsic().getLastModified()) {
        updateIntrinsicCalibration = true;
    }

    if (imuMessage->calib.lastModifiedExtrinsic != ecal_msg.getExtrinsic().getLastModified()) {
        updateExtrinsicCalibration = true;
    }

    // update imu intrinsic
    if (updateIntrinsicCalibration) {
        imuMessage->calib.lastModifiedIntrinsic = ecal_msg.getIntrinsic().getLastModified();
        std::cout << "received updated intrinsic for imu, ts = " << imuMessage->calib.lastModifiedExtrinsic << std::endl;

        const auto& intrinsicMsg = ecal_msg.getIntrinsic();

        imuMessage->calib.gyroNoiseStd = {
            intrinsicMsg.getGyroNoiseStd().getX(),
            intrinsicMsg.getGyroNoiseStd().getY(),
            intrinsicMsg.getGyroNoiseStd().getZ()
        };

        imuMessage->calib.accelNoiseStd = {
            intrinsicMsg.getAccelNoiseStd().getX(),
            intrinsicMsg.getAccelNoiseStd().getY(),
            intrinsicMsg.getAccelNoiseStd().getZ()
        };

        imuMessage->calib.gyroBiasStd = {
            intrinsicMsg.getGyroBiasStd().getX(),
            intrinsicMsg.getGyroBiasStd().getY(),
            intrinsicMsg.getGyroBiasStd().getZ()
        };

        imuMessage->calib.accelBiasStd = {
            intrinsicMsg.getAccelBiasStd().getX(),
            intrinsicMsg.getAccelBiasStd().getY(),
            intrinsicMsg.getAccelBiasStd().getZ()
        };

        imuMessage->calib.updateRate = intrinsicMsg.getUpdateRate();
        imuMessage->calib.timeOffsetNs = intrinsicMsg.getTimeOffsetNs();

    }

    // update imu extrinsic (body)
    if (updateExtrinsicCalibration) {
        imuMessage->calib.lastModifiedExtrinsic = ecal_msg.getExtrinsic().getLastModified();
        std::cout << "received updated extrinsic for imu, ts = " << imuMessage->calib.lastModifiedExtrinsic << std::endl;

        const auto& extrinsicMsg = ecal_msg.getExtrinsic();

        // body frame
        {
            Eigen::Vector3d position = {
            extrinsicMsg.getBodyFrame().getPosition().getX(),
            extrinsicMsg.getBodyFrame().getPosition().getY(),
            extrinsicMsg.getBodyFrame().getPosition().getZ()
            };

            Eigen::Quaterniond orientation = {
                extrinsicMsg.getBodyFrame().getOrientation().getW(),
                extrinsicMsg.getBodyFrame().getOrientation().getX(),
                extrinsicMsg.getBodyFrame().getOrientation().getY(),
                extrinsicMsg.getBodyFrame().getOrientation().getZ()
            };

            orientation.normalize();

            imuMessage->calib.body_T_imu.linear() = orientation.toRotationMatrix();
            imuMessage->calib.body_T_imu.translation() = position;

            std::cout << "body_T_imu: " << std::endl << imuMessage->calib.body_T_imu.affine() << std::endl;

        }
    }

    for (auto& callback : m_registeredImuCallbacks) {
        callback(imuMessage);
    }
    

}

void CameraInternal::sendCameraControl(const CameraControlData& data) {

    ecal::CameraControl::Builder msg = m_cameraControlPub->GetBuilder();

    std::uint64_t nowTns = std::chrono::steady_clock::now().time_since_epoch().count();

    if (msg.hasHeader()) {
        auto header = msg.getHeader();
    
        header.setStamp(nowTns);
        header.setSeq(header.getSeq() + 1);
    }else {
        auto header = msg.getHeader();
    
        header.setStamp(nowTns);
        header.setSeq(0);
    }

    msg.setStreaming(data.streaming);
    msg.setExposureUSec(data.exposureUSec);
    msg.setGain(data.gain);
    msg.setExposureCompensation(data.exposureCompensation);
    msg.setSensorIdx(data.sensorIdx);

    m_cameraControlPub->Send();

}

void CameraInternal::sendJsonIn(const std::string& topic, const std::string& content) {

}

CameraInternal::~CameraInternal()
{
    if (!m_ecalInitialisedOutside)
        eCAL::Finalize();
}


} // namespace vk