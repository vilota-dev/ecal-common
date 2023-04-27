#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

capnp.add_import_hook(['../src/capnp'])

import disparity_capnp as eCALDiaprity

imshow_map = {}


def callback(type, topic_name, msg, ts):

    # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
    with eCALDiaprity.Disparity.from_bytes(msg) as imageMsg:
        print(f"seq = {imageMsg.header.seq}, with {len(msg)} bytes, encoding = {imageMsg.encoding}")
        print(f"width = {imageMsg.width}, height = {imageMsg.height}")
        print(f"[fx fy cx cy baseline] - sensor = [{imageMsg.fx} {imageMsg.fy} {imageMsg.cx} {imageMsg.cy} {imageMsg.baseline}] - {imageMsg.streamName}")

        if (imageMsg.encoding == "disparity8"):

            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height, imageMsg.width, 1))

            disp = (mat * (255.0 / imageMsg.maxDisparity)).astype(np.uint8)
            disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

            imshow_map[topic_name + " diaprity8"] = disp
            # cv2.imshow("diaprity8", disp)
            # cv2.waitKey(3)
        elif (imageMsg.encoding == "disparity16"):
            mat_uint16 = np.frombuffer(imageMsg.data, dtype=np.uint16)
            mat_uint16 = mat_uint16.reshape((imageMsg.height, imageMsg.width, 1))

            mat_float32 = mat_uint16.astype(np.float32) / 8.0

            disp = (mat_float32 * (255.0 / imageMsg.maxDisparity)).astype(np.uint8)
            disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

            # mat = cv2.cvtColor(mat, cv2.COLOR_YUV2BGR_IYUV)
            imshow_map[topic_name + " disparity16"] = disp
            # cv2.imshow("disparity", disp)
            # cv2.waitKey(3)
        elif (imageMsg.encoding == "depth16"):
            mat_uint16 = np.frombuffer(imageMsg.data, dtype=np.uint16)
            mat_uint16 = mat_uint16.reshape((imageMsg.height, imageMsg.width, 1))

            mat_float32 = mat_uint16.astype(np.float32) / 1000 # convert millimeter to meter

            max_depth = 5 # in meters

            depth = (mat_float32 * (255.0 / max_depth)).astype(np.uint8)
            # depth = 255 - (mat_float32 * (255.0 / max_depth)).astype(np.uint8)

            # depth = np.where(depth == 255, 0 , depth)
            depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
            imshow_map[topic_name + " depth16"] = depth
            # cv2.imshow("depth", depth)
            # cv2.waitKey(3)


def main():  
    # mat = np.ones((800,1280,1), dtype=np.uint8) * 125
    # cv2.imshow("mono8", mat)
    # cv2.imwrite("test0.jpg", mat)

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_disparity_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    n = len(sys.argv)
    if n == 1:
        topic = "S0/disparity/stereo1"
    elif n == 2:
        topic = sys.argv[1]
    else:
        raise RuntimeError("Need to pass in exactly one parameter for topic")

    print(f"Streaming topic {topic}")

    # create subscriber and connect callback
    sub = CapnpSubscriber("Disparity", topic)
    # sub = ByteSubscriber("S0/depth/stereo1")
    sub.set_callback(callback)
    
    # idle main thread
    while ecal_core.ok():
        for im in imshow_map:
            cv2.imshow(im, imshow_map[im])
        cv2.waitKey(3)
        # time.sleep(0.1)
        
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()