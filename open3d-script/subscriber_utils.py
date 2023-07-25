import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber

capnp.add_import_hook(['../src/capnp'])

import odometry3d_capnp as eCALOdometry3d
import image_capnp as eCALImage
import imu_capnp as eCALImu
import tagdetection_capnp as eCALTagDetections
import path_capnp as eCALPath

import sys
sys.path.insert(0, "../python")
from utils import SyncedImageSubscriber

import time

import queue
from collections import namedtuple
from threading import Lock

class ImuSubscriber:
    def __init__(self, topic):
        print(f"subscribing to imu topic {topic}")
        sub = self.subscriber = ByteSubscriber(topic)
        sub.set_callback(self.callback)

        # self.lock = Lock()

        # self.warn_drop = False
        self.rolling = False

        self.m_queue = queue.Queue(2000)
        self.latest = None

        self.topic = topic

    # def queue_clear(self):
    #     with self.lock:
    #         self.m_queue = queue.Queue(200)

    def queue_update(self):

        if self.m_queue.full():
            if self.warn_drop is True:
                    print("imu queue is not processed in time")
            self.m_queue.get()
                

    def callback(self, topic_name, msg, ts):
        with eCALImu.Imu.from_bytes(msg) as imuMsg:

            self.latest = imuMsg

            if self.rolling:
                try:
                    self.m_queue.put(imuMsg, block=False)
                except queue.Full:
                    print("imu queue full")
                    # print(self.m_queue.qsize())
                    self.m_queue.get()
                    self.m_queue.put(imuMsg)

            # self.queue_update()

    def pop_latest(self):

        # with self.lock:
        if self.latest == None:
            return {}
        else:
            return self.latest

    def pop_queue(self):
        # imu is quite frequent, it is ok to be blocked

        # if self.m_queue.qsize() > 10:
        #     print(self.m_queue.qsize())
        return self.m_queue.get()

class AprilPathSubscriber:

    def __init__(self, path_topic):
        self.path_sub = ByteSubscriber(path_topic)
        self.path_sub.set_callback(self._path_callback)
        self.path_msg = None
        self.has_updated = False

    def get_path(self):
        if self.has_updated:
            self.has_updated = False
            return self.path_msg
        else:
            return None

    def _path_callback(self, topic_name, msg, time_ecal):
        print("PATH:", topic_name)
        with eCALPath.Path.from_bytes(msg) as pathMsg:
            self.path_msg = pathMsg
            self.has_updated = True

OdometryMsg = namedtuple("OdometryMsg", ["ts", 
                                         "header",
                                         "position_x", 
                                         "position_y", 
                                         "position_z", 
                                         "orientation_x",
                                         "orientation_y",
                                         "orientation_z",
                                         "orientation_w"])

class VioSubscriber:
    def __init__(self, vio_topic):

        self.vio_sub = ByteSubscriber(vio_topic)
        self.vio_msg = ""
        self.vio_sub.set_callback(self._vio_callback)

        self.vio_callbacks = []

        self.header = None
        self.odometry_msg = None

    def register_callback(self, cb):
        self.vio_callbacks.append(cb)

    def _vio_callback(self, topic_name, msg, time_ecal):
        # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
                
        with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:

            self.odometry_msg = OdometryMsg(ts=odometryMsg.header.stamp,
                                            header=odometryMsg.header,
                                            position_x=odometryMsg.pose.position.x,
                                            position_y=odometryMsg.pose.position.y,
                                            position_z=odometryMsg.pose.position.z,
                                            orientation_x=odometryMsg.pose.orientation.x,
                                            orientation_y=odometryMsg.pose.orientation.y,
                                            orientation_z=odometryMsg.pose.orientation.z,
                                            orientation_w=odometryMsg.pose.orientation.w)

            for cb in self.vio_callbacks:
                cb("capnp:Odometry3D", topic_name, self.odometry_msg, time_ecal)

            # text
            self.header = odometryMsg.header
            position_msg = f"position: \n {odometryMsg.pose.position.x:.4f}, {odometryMsg.pose.position.y:.4f}, {odometryMsg.pose.position.z:.4f}"
            orientation_msg = f"orientation: \n  {odometryMsg.pose.orientation.w:.4f}, {odometryMsg.pose.orientation.x:.4f}, {odometryMsg.pose.orientation.y:.4f}, {odometryMsg.pose.orientation.z:.4f}"
            
            device_latency_msg = f"device latency = {odometryMsg.header.latencyDevice / 1e6 : .2f} ms"
            
            vio_host_latency = time.monotonic() *1e9 - odometryMsg.header.stamp 
            host_latency_msg = f"host latency = {vio_host_latency / 1e6 :.2f} ms"
            
            self.vio_msg = position_msg + "\n" + orientation_msg + "\n" + device_latency_msg + "\n" + host_latency_msg



class TagDetectionsSubscriber:
    lock = Lock()

    def __init__(self, tags_topic):
        self.uses_vio = False

        self.types = ["TagDetections"]
        self.topics = [tags_topic]
        self.typeclasses = [eCALTagDetections.TagDetections]

        self.tags_sub = SyncedImageSubscriber(self.types, self.topics, self.typeclasses, enforce_sync=True)
        self.tags_sub.register_callback(self.callback)

        self.topic = tags_topic
        self.tags = None
        self.vio = None
        self.new_data = False
        self.tag_geometries = set()
        self.tag_labels = set()

    def add_vio_sub(self, vio_sub, vio_topic):
        self.uses_vio = True
        self.types.append("Odometry3d")
        self.topics.append(vio_topic)
        self.typeclasses.append(eCALOdometry3d.Odometry3d)
        self.tags_sub.add_external_sub(vio_sub,
                                       vio_topic)

    def callback(self, msg, ts):
        with TagDetectionsSubscriber.lock:
            self.tags = msg[self.topics[0]]
            self.vio = msg[self.topics[1]] if self.uses_vio else None
            self.new_data = True
            print(f"received message with {len(self.tags.tags)} tags at {self.tags.header.stamp}:")
            for tag in self.tags.tags:
                pose = tag.poseInCameraFrame.position
                print(f"\ttag {tag.id} at {pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f}")


def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation = inter)

    # return the resized image
    return resized




def add_ui_on_ndarray(img_ndarray, exposureUSec, gain, latencyDevice_display, latencyHost_display, expMax, sensIsoMax, expTime_display_flag, sensIso_display_flag, latencyDevice_display_flag,latencyHost_display_flag):
    
    expMin = 10
    sensMin = 100
    
    progress_bar_length = 200
    progress_bar_height = 15
    
    left_x = 10
    
    left_y_pb1 = 30
    spacing_2pb = 30
    left_y_pb2 = left_y_pb1 + progress_bar_height + spacing_2pb

    expTime_frame_start = (left_x, left_y_pb1)
    expTime_frame_end = (left_x + progress_bar_length, left_y_pb1 + progress_bar_height)
    
    sensIso_frame_start = (left_x,left_y_pb2)
    sensIso_frame_end = (left_x + progress_bar_length, left_y_pb2 + progress_bar_height)

    spacing_pb_text = 40
    spacing_2text = 20
    latencyDevice_coor = (left_x, left_y_pb2 + progress_bar_height + spacing_pb_text)
    latencyHost_coor = (left_x, left_y_pb2 + progress_bar_height + spacing_pb_text + spacing_2text)

    if expTime_display_flag:
        # add progress bar
        expTime_length = int((exposureUSec - expMin) / (expMax - expMin) * progress_bar_length)
        img_ndarray = cv2.rectangle(img_ndarray, expTime_frame_start, (left_x + expTime_length, expTime_frame_end[1]), (255, 0, 0),-1)
        # add progress bar frame
        img_ndarray = cv2.rectangle(img_ndarray, expTime_frame_start, expTime_frame_end, (255, 255, 255),2)
        # add description
        img_ndarray = cv2.putText(img_ndarray, 'expTime', (expTime_frame_end[0]+5, expTime_frame_start[1]), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0), 1)
        img_ndarray = cv2.putText(img_ndarray, str(expMin), (expTime_frame_start[0],expTime_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    
        img_ndarray = cv2.putText(img_ndarray, str(expMax), (expTime_frame_end[0],expTime_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    

    if sensIso_display_flag:
        # add progress bar
        sensIso_length = int((gain - sensMin) / (sensIsoMax - sensMin) * progress_bar_length)
        img_ndarray = cv2.rectangle(img_ndarray, sensIso_frame_start, (left_x +sensIso_length, sensIso_frame_end[1]), (0, 0, 255),-1)
        # add progress bar frame
        img_ndarray = cv2.rectangle(img_ndarray, sensIso_frame_start, sensIso_frame_end, (255, 255, 255),2)
        # add description
        img_ndarray = cv2.putText(img_ndarray, 'sensIso', (sensIso_frame_end[0]+5, sensIso_frame_start[1]), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255), 1)    
        img_ndarray = cv2.putText(img_ndarray, str(sensMin), (sensIso_frame_start[0],sensIso_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    
        img_ndarray = cv2.putText(img_ndarray, str(sensIsoMax), (sensIso_frame_end[0],sensIso_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    


    # add latency text
    if latencyDevice_display_flag:
        img_ndarray = cv2.putText(img_ndarray, latencyDevice_display, latencyDevice_coor, cv2.FONT_HERSHEY_TRIPLEX, 0.5, (127, 0, 255), 1)  
    
    if latencyHost_display_flag:
        img_ndarray = cv2.putText(img_ndarray, latencyHost_display, latencyHost_coor, cv2.FONT_HERSHEY_TRIPLEX, 0.5, (127, 0, 255), 1)

    return img_ndarray    

def is_numeric(input_str):
    if input_str == "":
        return False
    else:
        for char in input_str:
            if not char.isnumeric():
                return False
        return True