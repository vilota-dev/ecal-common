#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber

capnp.add_import_hook(['../src/capnp'])

import detection2d_capnp as eCALDetection2d

imshow_map = {}


# nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
def frameNorm(frame, bbox):
    normVals = np.full(len(bbox), frame.shape[0])
    normVals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

def displayFrame(frame, detections, labels):
    color = (255, 0, 0)
    for detection in detections:
        bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
        print(f"{labels[detection.labelIdx]} : {bbox}")
        cv2.putText(frame, labels[detection.labelIdx], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)

def callback(topic_name, msg, ts):

    with eCALDetection2d.Detections2d.from_bytes(msg) as d:
        print(f"seq = {d.header.seq}, with {len(msg)} bytes")
        print(f"width = {d.image.width}, height = {d.image.height}")

        if (d.image.encoding == "mono8"):

            mat = np.frombuffer(d.image.data, dtype=np.uint8)
            mat = mat.reshape((d.image.height, d.image.width, 1))
            mat = cv2.cvtColor(mat, cv2.COLOR_GRAY2BGR)

        elif (d.image.encoding == "yuv420"):
            mat = np.frombuffer(d.image.data, dtype=np.uint8)
            mat = mat.reshape((d.image.height * 3 // 2, d.image.width, 1))
            mat = cv2.cvtColor(mat, cv2.COLOR_YUV2BGR_IYUV)

        elif (d.image.encoding == "bgr8"):
            mat = np.frombuffer(d.image.data, dtype=np.uint8)
            mat = mat.reshape((d.image.height, d.image.width, 3))

        elif (d.image.encoding == "jpeg"):
            mat_jpeg = np.frombuffer(d.image.data, dtype=np.uint8)
            mat = cv2.imdecode(mat_jpeg, cv2.IMREAD_COLOR)

        else:
            raise RuntimeError("unknown encoding: " + d.image.encoding)


        displayFrame(mat, d.detections, d.labels)
        # bbox = np.zeros(4)
        # bbox[:] = [msg.xmin, msg.ymin, msg.xmax, msg.ymax]

        imshow_map["detection2d"] = mat


def main():  

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_image_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    n = len(sys.argv)
    if n == 1:
        topic = "S0/yolo/camb"
    elif n == 2:
        topic = sys.argv[1]
    else:
        raise RuntimeError("Need to pass in exactly one parameter for topic")

    print(f"Streaming topic {topic}")

    # create subscriber and connect callback
    sub = ByteSubscriber(topic)
    # sub = ByteSubscriber("raw_fisheye_image")
    # sub = ByteSubscriber("S0/stereo1_l")
    sub.set_callback(callback)
    
    # idle main thread
    while ecal_core.ok():
        for im in imshow_map:
            cv2.imshow(im, imshow_map[im])
        cv2.waitKey(10)
        
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()