import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage
import disparity_capnp as eCALDisparity
import imu_capnp as eCALImu

import queue
from threading import Lock

class ImuSubscriber:
    def __init__(self, topic):
        print(f"subscribing to imu topic {topic}")
        sub = self.subscriber = CapnpSubscriber("Imu", topic)
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
                

    def callback(self, topic_type, topic_name, msg, ts):
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
    def __init__(self, types, topics):

        self.subscribers = {}
        self.queues = {} # store individual incoming images
        self.synced_queue = queue.Queue(150) # store properly synced images
        self.latest = None
        assert len(types) == len(topics)
        self.size = len(topics)

        # self.warn_drop = False

        self.rolling = False

        self.assemble = {}
        self.assemble_index = -1

        self.lock = Lock()

        for i in range(len(topics)):
            print(f"subscribing to {types[i]} topic {topics[i]}")
            sub = self.subscribers[topics[i]] = CapnpSubscriber(types[i], topics[i])
            sub.set_callback(self.callback)

            self.queues[topics[i]] = queue.Queue(10)

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
                    break

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

                

                


    def callback(self, topic_type, topic_name, msg, ts):
            # need to remove the .decode() function within the Python API of ecal.core.subscriber StringSubscriber

        if topic_type == "capnp:Image":
            with eCALImage.Image.from_bytes(msg) as imageMsg:

                self.queues[topic_name].put(imageMsg)

                with self.lock:
                    self.queue_update()
        elif topic_type == "capnp:Disparity":
            with eCALDisparity.Disparity.from_bytes(msg) as disparityMsg:

                self.queues[topic_name].put(disparityMsg)

                with self.lock:
                    self.queue_update()
        else:
            raise RuntimeError(f"unknown type received: {topic_type}")

    def pop_latest(self):

        with self.lock:

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

def image_msg_to_cv_mat(imageMsg):
    if (imageMsg.encoding == "mono8"):

        mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
        mat = mat.reshape((imageMsg.height, imageMsg.width, 1))

        mat_vis = cv2.cvtColor(mat, cv2.COLOR_GRAY2BGR)

        return mat, mat_vis
    elif (imageMsg.encoding == "yuv420"):
        mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
        mat = mat.reshape((imageMsg.height * 3 // 2, imageMsg.width, 1))

        mat = cv2.cvtColor(mat, cv2.COLOR_YUV2BGR_IYUV)

        return mat, mat
    elif (imageMsg.encoding == "bgr8"):
        mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
        mat = mat.reshape((imageMsg.height, imageMsg.width, 3))
        return mat, mat
    elif (imageMsg.encoding == "jpeg"):
        mat_jpeg = np.frombuffer(imageMsg.data, dtype=np.uint8)
        mat = cv2.imdecode(mat_jpeg, cv2.IMREAD_COLOR)
        return mat, mat
    else:
        raise RuntimeError("unknown encoding: " + imageMsg.encoding)
    
def disparity_to_cv_mat(imageMsg):
    if (imageMsg.encoding == "disparity8"):

        mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
        mat = mat.reshape((imageMsg.height, imageMsg.width, 1))

        disp_vis = (mat * (255.0 / imageMsg.maxDisparity)).astype(np.uint8)
        disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        return mat, disp_vis
    
    elif (imageMsg.encoding == "disparity16"):
        mat_uint16 = np.frombuffer(imageMsg.data, dtype=np.uint16)
        mat_uint16 = mat_uint16.reshape((imageMsg.height, imageMsg.width, 1))

        mat_float32 = mat_uint16.astype(np.float32) / 8.0

        disp_vis = (mat_float32 * (255.0 / imageMsg.maxDisparity)).astype(np.uint8)
        disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        return mat_float32, disp_vis
    else:
        raise RuntimeError(f"disparity type not supported: {imageMsg.encoding}")