import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber

capnp.add_import_hook(['../src/capnp'])

import odometry3d_capnp as eCALOdometry3d
import image_capnp as eCALImage
import imu_capnp as eCALImu

import queue
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



class SyncedImageSubscriber:
    def __init__(self, topics):

        self.subscribers = {}
        self.queues = {} # store individual incoming images
        self.synced_queue = queue.Queue(150) # store properly synced images
        self.latest = None
        self.size = len(topics)

        # self.warn_drop = False

        self.rolling = False

        self.assemble = {}
        self.assemble_index = -1

        # self.lock = Lock()

        for topic in topics:
            print(f"subscribing to image topic {topic}")
            sub = self.subscribers[topic] = ByteSubscriber(topic)
            sub.set_callback(self.callback)

            self.queues[topic] = queue.Queue(10)

    # def queue_clear(self):
    #     with self.lock:

    #         for queueName in self.queues:

    #             queue = self.queues[queueName]
    #             queue = queue.Queue(30)

    def queue_update(self):

        # with self.lock:

        for queueName in self.queues:

            m_queue = self.queues[queueName]

            # already in assemble, no need to get from queue
            if queueName in self.assemble:
                continue

            while True:

                if m_queue.empty():
                    return

                imageMsg = m_queue.get()

                if self.assemble_index < imageMsg.header.stamp:
                    # we shall throw away the assemble and start again
                    
                    if self.assemble_index != -1:
                        print(f"reset index to {imageMsg.header.stamp}")

                    self.assemble_index = imageMsg.header.stamp
                    self.assemble = {}
                    self.assemble[queueName] = imageMsg
                    
                    continue
                elif self.assemble_index > imageMsg.header.stamp:
                    print(f"ignore {queueName} for later")
                    continue
                else:
                    self.assemble[queueName] = imageMsg
                    break

        
        # check for full assembly

        if len(self.assemble) == self.size:
            
            self.latest = self.assemble

            if self.rolling:
                try:
                    self.synced_queue.put(self.assemble, block=False)
                except queue.Full:
                    print("image queue full")
                    self.synced_queue.get()
                    self.synced_queue.put(self.assemble)
            

            self.assemble = {}

            # print(f"success assembling {self.assemble_index / 1.e9}")

            self.assemble_index = -1

            # if self.synced_queue.full():
            #     if self.warn_drop is True:
            #         print("image sync_queue is not processed in time")
            #     self.synced_queue.get()

                

                


    def callback(self, topic_name, msg, ts):
            # need to remove the .decode() function within the Python API of ecal.core.subscriber StringSubscriber
        with eCALImage.Image.from_bytes(msg) as imageMsg:
            # print(f"{topic_name} seq = {imageMsg.header.seq}, with {len(msg)} bytes, encoding = {imageMsg.encoding}")
            # print(f"width = {imageMsg.width}, height = {imageMsg.height}")
            # print(f"exposure = {imageMsg.exposureUSec}, gain = {imageMsg.gain}")

            self.queues[topic_name].put(imageMsg)

            self.queue_update()

    def pop_latest(self):

        # with self.lock:

        if self.latest == None:
            return {}
        else:
            return self.latest

    def pop_sync_queue(self):
        # not protected for read

        return self.synced_queue.get()

        # try:
        #     return self.synced_queue.get(False)
        # except queue.Empty:
        #     return None



class AsyncedImageSubscriber:
    def __init__(self, topics):

        self.subscribers = {}
        self.queues = {} # store individual incoming images
        self.asynced_queue = queue.Queue(150) # store properly synced images
        self.latest = None
        self.size = len(topics)

        # self.warn_drop = False

        self.rolling = False

        self.assemble = {}

        # self.lock = Lock()

        for topic in topics:
            print(f"subscribing to image topic {topic}")
            sub = self.subscribers[topic] = ByteSubscriber(topic)
            sub.set_callback(self.callback)

            self.queues[topic] = queue.Queue(10)


    def queue_update(self):

        # with self.lock:

        for queueName in self.queues:

            m_queue = self.queues[queueName]

            # already in assemble, no need to get from queue
            if queueName in self.assemble:
                continue

            while True:

                if m_queue.empty():
                    return

                imageMsg = m_queue.get()
                self.assemble[queueName] = imageMsg
                

        
        # check for full assembly

        if len(self.assemble) == self.size:
            
            self.latest = self.assemble

            if self.rolling:
                try:
                    self.asynced_queue.put(self.assemble, block=False)
                except queue.Full:
                    print("image queue full")
                    self.asynced_queue.get()
                    self.asynced_queue.put(self.assemble)
            

            self.assemble = {}



                

    def callback(self, topic_name, msg, ts):
            # need to remove the .decode() function within the Python API of ecal.core.subscriber StringSubscriber
        with eCALImage.Image.from_bytes(msg) as imageMsg:

            self.queues[topic_name].put(imageMsg)

            self.queue_update()

    def pop_latest(self):

        # with self.lock:

        if self.latest == None:
            return {}
        else:
            return self.latest

    def pop_async_queue(self):
        # not protected for read

        return self.asynced_queue.get()

        # try:
        #     return self.synced_queue.get(False)
        # except queue.Empty:
        #     return None



class VioSubscriber:

    def __init__(self, vio_topic):

        self.vio_sub = ByteSubscriber(vio_topic)
        self.vio_msg = ""
        self.vio_sub.set_callback(self._vio_callback)

    def _vio_callback(self, topic_name, msg, time):

        # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
        
        with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:

            # if first_message:
            #     print(f"bodyFrame = {odometryMsg.bodyFrame}")
            #     print(f"referenceFrame = {odometryMsg.referenceFrame}")
            #     print(f"velocityFrame = {odometryMsg.velocityFrame}")
            #     first_message = False

            position_msg = f"position: \n {odometryMsg.pose.position.x:.4f}, {odometryMsg.pose.position.y:.4f}, {odometryMsg.pose.position.z:.4f}"
            orientation_msg = f"orientation: \n  {odometryMsg.pose.orientation.w:.4f}, {odometryMsg.pose.orientation.x:.4f}, {odometryMsg.pose.orientation.y:.4f}, {odometryMsg.pose.orientation.z:.4f}"
            
            device_latency_msg = f"device latency = {odometryMsg.header.latencyDevice / 1e6 : .2f} ms"
            
            vio_host_latency = monotonic() *1e9 - odometryMsg.header.stamp 
            host_latency_msg = f"host latency = {vio_host_latency / 1e6 :.2f} ms"
            
            self.vio_msg = position_msg + "\n" + orientation_msg + "\n" + device_latency_msg + "\n" + host_latency_msg




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




def add_ui_on_ndarray(img_ndarray, exposureUSec, gain, latencyDevice_display, latencyHost_display, expTime_display_flag, sensIso_display_flag, latencyDevice_display_flag,latencyHost_display_flag):
    
    expMin = 10
    expMax = 12000
    sensMin = 100
    sensMax = 800
    
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
        sensIso_length = int((gain - sensMin) / (sensMax - sensMin) * progress_bar_length)
        img_ndarray = cv2.rectangle(img_ndarray, sensIso_frame_start, (left_x +sensIso_length, sensIso_frame_end[1]), (0, 0, 255),-1)
        # add progress bar frame
        img_ndarray = cv2.rectangle(img_ndarray, sensIso_frame_start, sensIso_frame_end, (255, 255, 255),2)
        # add description
        img_ndarray = cv2.putText(img_ndarray, 'sensIso', (sensIso_frame_end[0]+5, sensIso_frame_start[1]), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255), 1)    
        img_ndarray = cv2.putText(img_ndarray, str(sensMin), (sensIso_frame_start[0],sensIso_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    
        img_ndarray = cv2.putText(img_ndarray, str(sensMax), (sensIso_frame_end[0],sensIso_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    


    # add latency text
    if latencyDevice_display_flag:
        img_ndarray = cv2.putText(img_ndarray, latencyDevice_display, latencyDevice_coor, cv2.FONT_HERSHEY_TRIPLEX, 0.5, (127, 0, 255), 1)  
    
    if latencyHost_display_flag:
        img_ndarray = cv2.putText(img_ndarray, latencyHost_display, latencyHost_coor, cv2.FONT_HERSHEY_TRIPLEX, 0.5, (127, 0, 255), 1)

    return img_ndarray    

