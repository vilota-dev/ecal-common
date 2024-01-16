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

    format = ''
    for field in msg.fields:
        if field.type == "float32":
            format += 'f'
        elif field.type == "float64":
            format += 'd'
        elif field.type == "int8":
            format += 'b'
        elif field.type == "uint8":
            format += 'B'
        elif field.type == "int16":
            format += 'h'
        elif field.type == "uint16":
            format += 'H'
        elif field.type == "int32":
            format += 'i'
        elif field.type == "uint32":
            format += 'I'
        elif field.type == "int64":
            format += 'q'
        elif field.type == "uint64":
            format += 'Q'
    format_size = struct.calcsize(format)

    arr = [struct.unpack(format, data[i:i+format_size]) for i in range(0, len(data), format_size)]

    print (f"Received {len(arr)} points @ {msg.header.stamp}")
    print (f"data = {arr}")
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
