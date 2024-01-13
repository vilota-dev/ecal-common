#!/usr/bin/env python3

import sys
import time
import struct

import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

capnp.add_import_hook(['../thirdparty/vk_common/capnp'])

import pointcloud_capnp as eCALPointCloud

def callback(type, topic_name, msg, ts):
    data = msg.points

    arr = [struct.unpack('fffBB', data[i:i+struct.calcsize('fffBB')]) for i in range(0, len(data), struct.calcsize('fffBB'))]

    print (arr)
    print("===================================")

def main():  
    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_image_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    # create subscriber and connect callback

    n = len(sys.argv)
    if n == 1:
        topic = "S0/tag_pcl"
    elif n == 2:
        topic = sys.argv[1]
    else:
        raise RuntimeError("Need to pass in exactly one parameter for topic")

    print(f"Streaming topic {topic}")
    sub = CapnpSubscriber("Image", topic, eCALPointCloud.PointCloud)
    sub.set_callback(callback)
    
    # idle main thread
    while ecal_core.ok():
        time.sleep(0.1)        
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()
