import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

capnp.add_import_hook(['../thirdparty/vk_common/capnp'])

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
    def __init__(self, types, topics, 
                 typeclasses=None, 
                 enforce_sync=True):

        self.subscribers = {}
        # store individual incoming images
        self.queues = {}
        # store properly synced images
        self.synced_queue = queue.Queue(150)
        self.enforce_sync = enforce_sync
        self.callbacks = []
        
        self.latest = None
        assert len(types) == len(topics)
        assert typeclasses is None or len(typeclasses) == len(topics)
        self.size = len(topics)

        self.rolling = False

        self.assemble = {}
        self.assemble_index = -1

        self.lock = Lock()

        for i in range(len(topics)):
            print(f"subscribing to {types[i]} topic {topics[i]}")
            typeclass = typeclasses[i] if typeclasses is not None else None
            sub = self.subscribers[topics[i]] = CapnpSubscriber(types[i], topics[i], typeclass)
            sub.set_callback(self.callback)

            self.queues[topics[i]] = queue.Queue(10)

    def queue_update(self):
        for queueName in self.queues:
            m_queue = self.queues[queueName]

            # already in assemble, no need to get from queue
            if queueName in self.assemble:
                continue

            while True:                  

                if m_queue.empty():
                    break

                imageMsg = m_queue.get()

                if self.enforce_sync and self.assemble_index < imageMsg.header.stamp:
                    # we shall throw away the assemble and start again
                    if self.assemble_index != -1:
                        print(f"reset index to {imageMsg.header.stamp}")

                    self.assemble_index = imageMsg.header.stamp
                    self.assemble = {}
                    self.assemble[queueName] = imageMsg
                    
                    continue
                elif self.enforce_sync and self.assemble_index > imageMsg.header.stamp:
                    # print(f"ignore {queueName} for later")
                    continue
                else:
                    self.assemble[queueName] = imageMsg
                    if self.enforce_sync:
                        break        
        
        # check for full assembly
        if len(self.assemble) == self.size:
            self.latest = self.assemble

            for cb in self.callbacks:
                cb(self.latest, self.assemble_index)

            if self.rolling:
                if self.synced_queue.full():
                    print(f"queue full: {self.queues.keys()}")
                    self.synced_queue.get()
                    self.synced_queue.put(self.assemble)
                else:
                    self.synced_queue.put(self.assemble, block=False)
            self.assemble = {}
            self.assemble_index = -1

    def callback(self, topic_type, topic_name, msg, ts):
        if self.queues[topic_name].full():
            self.queues[topic_name].get()
        
        self.queues[topic_name].put(msg)

        with self.lock:
            self.queue_update()

    # sub must have a `register_callback()` method
    def add_external_sub(self, sub, topic_name):
        self.queues[topic_name] = queue.Queue(10)
        self.size += 1
        sub.register_callback(self.callback)

    def register_callback(self, cb):
        self.callbacks.append(cb)

    def pop_latest(self):
        with self.lock:
            if self.latest == None:
                return {}
            else:
                return self.latest

    def pop_sync_queue(self):
        # not protected for read
        return self.synced_queue.get()

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