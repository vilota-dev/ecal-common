#!/usr/bin/env python3

import sys
import time
from datetime import datetime

import signal

import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
# from byte_subscriber import ByteSubscriber

# capnp.add_import_hook(['../src/capnp'])

# import image_capnp as eCALImage

from utils import SyncedImageSubscriber, ImuSubscriber, image_resize

from rospy import rostime
import rosbag
from sensor_msgs.msg import Image, Imu

import threading


from enum import Enum

class DatasetMode(Enum):
    CONTINUOUS = 1
    SNAPSHOTS = 2

class RosbagDatasetRecorder:

    def __init__(self, image_topics, imu_topic, bag_name, mode):

        self.image_sub = SyncedImageSubscriber(image_topics)

        if mode == DatasetMode.CONTINUOUS:
            self.imu_sub = ImuSubscriber(imu_topic)
        else:
            self.imu_sub = None

        self.bag = None
        self.bag_name = bag_name
        self.mode = mode

        self.recording = False
        self.thread = None

        self.num_snapshot = 0

        self.last_imu_seq = -1

        self.bag_lock = threading.Lock()


    def start_record(self):

        if self.bag is None:
            self.bag = rosbag.Bag(self.bag_name, mode='w', compression=rosbag.Compression.NONE)

        if self.mode == DatasetMode.CONTINUOUS and self.recording is False:
            self.thread_image = threading.Thread(target=self.continuous_recording_thread_image)
            self.thread_imu = threading.Thread(target=self.continuous_recording_thread_imu)
            self.recording = True
            self.image_sub.rolling = True
            self.imu_sub.rolling = True
            self.thread_image.start()
            self.thread_imu.start()

        
        if self.mode == DatasetMode.SNAPSHOTS:
            
            # only capture images
            image_dict = self.image_sub.pop_latest()

            for imageName in image_dict:
                imageMsg = image_dict[imageName]
                imgMsg, stamp = self.ecal2ros_image(imageMsg)
                self.bag.write(imageName, imgMsg, stamp)

            self.num_snapshot += 1

            print(f"snapshot taken {self.num_snapshot}")

    def stop_record(self):

        if self.mode == DatasetMode.CONTINUOUS:
            self.recording = False

            if self.thread_image is not None:
                self.thread_image.join()
                self.thread_imu.join()
            
            print("stop record")

        if self.bag is not None:
            self.bag.close()
            print("bag file closes")

    def ecal2ros_image(self, imageMsg):

        imgMsg = Image()
        stamp = rostime.Time(int(imageMsg.header.stamp // 1e9), imageMsg.header.stamp % 1e9)
        imgMsg.header.stamp = stamp
        imgMsg.width = imageMsg.width
        imgMsg.height = imageMsg.height
        imgMsg.encoding = str(imageMsg.encoding)
        imgMsg.data = imageMsg.data

        return imgMsg, stamp

    def ecal2ros_imu(self, imuMsg):
        imu = Imu()
        stamp = rostime.Time(int(imuMsg.header.stamp // 1e9), imuMsg.header.stamp % 1e9)

        imu.header.stamp = stamp
        imu.angular_velocity.x = imuMsg.angularVelocity.x
        imu.angular_velocity.y = imuMsg.angularVelocity.y
        imu.angular_velocity.z = imuMsg.angularVelocity.z

        imu.linear_acceleration.x = imuMsg.linearAcceleration.x
        imu.linear_acceleration.y = imuMsg.linearAcceleration.y
        imu.linear_acceleration.z = imuMsg.linearAcceleration.z

        return imu, stamp

    def continuous_recording_thread_image(self):

        print("start recording image...")

        while self.recording:

            images = self.image_sub.pop_sync_queue()

            if images is not None:
                for imageName in images:
                    imageMsg = images[imageName]
                    imgMsg, stamp = self.ecal2ros_image(imageMsg)

                    with self.bag_lock:
                        # this may block for a while..
                        self.bag.write(imageName, imgMsg, stamp)
        
        print("continuous_recording_thread_image stopping")

           
    def continuous_recording_thread_imu(self):

        print("start recording imu...")

        while self.recording:

            imu_item = self.imu_sub.pop_queue()

            if self.last_imu_seq >= 0:
                if self.last_imu_seq + 1 != imu_item.header.seq:
                    print("imu jump detected for bag writing")
                    print(f"expect {self.last_imu_seq + 1} , gotten {imu_item.header.seq}")

            self.last_imu_seq = imu_item.header.seq
            imuMsg, stamp = self.ecal2ros_imu(imu_item)

            with self.bag_lock:
                # this may block for a while..
                self.bag.write(self.imu_sub.topic, imuMsg, stamp)

        print("continuous_recording_thread_imu stopping")



def main(mode): 

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "calibration_dataset_recorder")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    image_topics = ["S0/camc", "S0/cama"]
    imu_topic = "S0/imu"

    if mode == DatasetMode.SNAPSHOTS:
        mode_type = "snapshot"
    else:
        mode_type = "continous"
    
    bag_name = "./rosbag/" + datetime.now().strftime("%Y-%m-%d-%I-%M-%S-") + mode_type + "_recording.bag"

    recorder = RosbagDatasetRecorder(image_topics, imu_topic, bag_name , mode)

    def handler(signum, frame):
        print("ctrl-c is pressed")
        recorder.stop_record()
        exit(1)

    signal.signal(signal.SIGINT, handler)

    time.sleep(1)

    print(f"Capture mode = {mode}")
    print("spacebar to start snapshot / recording, `q` to stop and quit")

    while ecal_core.ok():
        image_dict = recorder.image_sub.pop_latest()

        image_list = []
        image_name = ""

        group = 0
        idx = 0

        for imageName in image_dict:

            image_name += imageName + " "

            imageMsg = image_dict[imageName]
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height, imageMsg.width, 1))
            mat_resized = image_resize(mat, width=600)

            image_list.append(mat_resized)

            idx += 1

            if idx == 3: # set maximum 3 images in a preview window
                image_name = f"[group {group}] " + image_name

                cv2.imshow(image_name, cv2.hconcat(image_list))

                group += 1
                image_name = ""
                image_list = []

        if (len(image_list)):
            cv2.imshow(f"[group {group}] " + image_name, cv2.hconcat(image_list))

        key = cv2.waitKey(10)
        if key == ord('q'):
            recorder.stop_record()
            break
        elif key == ord(' '):
            recorder.start_record()

    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":

    # main(DatasetMode.SNAPSHOTS) # for camera calibration
    main(DatasetMode.CONTINUOUS) # for camera-imu calibration

    # NOTE: for continuous mode, we have to make sure eCAL is configured optimally
    # https://github.com/eclipse-ecal/ecal/issues/869#issuecomment-1304970327
    # memfile_ack_timeout       = 10                                     
    # memfile_buffer_count      = 2