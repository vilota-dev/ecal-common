#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage

imshow_map = {}

def addStats(image, imageMsg):
    brightness = imageMsg.mipMapBrightness
    brightnessChange = imageMsg.mipMapBrightnessChange

    # Define the text and font properties
    text = f"b: {brightness} c:{brightnessChange:.2f}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_thickness = 1
    font_color = (255, 255, 255)  # White color

    text_position = (10, 30)
    cv2.putText(image, text, text_position, font, font_scale, font_color, font_thickness)


def callback(type, topic_name, msg, ts):

    # need to remove the .decode() function within the Python API of ecal.core.subscriber StringSubscriber
    with eCALImage.Image.from_bytes(msg) as imageMsg:
        print(f"seq = {imageMsg.header.seq}, stamp = {imageMsg.header.stamp}, with {len(msg)} bytes, encoding = {imageMsg.encoding}")
        # print(f"latency device = {imageMsg.header.latencyDevice / 1e6} ms")
        # print(f"latency host = {imageMsg.header.latencyHost / 1e6} ms")
        print(f"width = {imageMsg.width}, height = {imageMsg.height}")
        print(f"exposure = {imageMsg.exposureUSec}, gain = {imageMsg.gain}")
        # print(f"intrinsic = {imageMsg.intrinsic}")
        # print(f"extrinsic = {imageMsg.extrinsic}")
        print(f"instant w = {imageMsg.motionMeta.instantaneousAngularVelocity}")
        print(f"average w = {imageMsg.motionMeta.averageAngularVelocity}")

        if (imageMsg.encoding == "mono8"):

            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height, imageMsg.width, 1))

            addStats(mat, imageMsg)

            imshow_map[topic_name + " mono8"] = mat

            # cv2.imshow("mono8", mat)
            # cv2.waitKey(3)
        elif (imageMsg.encoding == "yuv420"):
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height * 3 // 2, imageMsg.width, 1))

            mat = cv2.cvtColor(mat, cv2.COLOR_YUV2BGR_IYUV)

            imshow_map[topic_name + " yuv420"] = mat
            # cv2.imshow("yuv420", mat)
            # cv2.waitKey(3)
        elif (imageMsg.encoding == "bgr8"):
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height, imageMsg.width, 3))
            imshow_map[topic_name + " bgr8"] = mat
        elif (imageMsg.encoding == "jpeg"):
            mat_jpeg = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = cv2.imdecode(mat_jpeg, cv2.IMREAD_COLOR)
            imshow_map[topic_name + " jpeg"] = mat
        else:
            raise RuntimeError("unknown encoding: " + imageMsg.encoding)


def main():  
    # mat = np.ones((800,1280,1), dtype=np.uint8) * 125
    # cv2.imshow("mono8", mat)
    # cv2.imwrite("test0.jpg", mat)

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_image_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    # create subscriber and connect callback

    n = len(sys.argv)
    if n == 1:
        topic = "S0/camd"
    elif n == 2:
        topic = sys.argv[1]
    else:
        raise RuntimeError("Need to pass in exactly one parameter for topic")

    print(f"Streaming topic {topic}")
    sub = CapnpSubscriber("Image", topic)
    sub.set_callback(callback)
    
    # idle main thread
    while ecal_core.ok():
        for im in imshow_map:
            cv2.imshow(im, imshow_map[im])
        cv2.waitKey(10)
        # time.sleep(0.01)
        
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()
