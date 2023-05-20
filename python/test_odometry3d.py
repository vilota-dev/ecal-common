#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

capnp.add_import_hook(['../src/capnp'])

import odometry3d_capnp as eCALOdometry3d


first_message = True

def callback(type, topic_name, msg, time):

    global first_message

    # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
    
    with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:
        print(f"seq = {odometryMsg.header.seq}")
        print(f"latency device = {odometryMsg.header.latencyDevice / 1e6} ms")
        print(f"latency host = {odometryMsg.header.latencyHost / 1e6} ms")

        if first_message:
            print(f"bodyFrame = {odometryMsg.bodyFrame}")
            print(f"referenceFrame = {odometryMsg.referenceFrame}")
            print(f"velocityFrame = {odometryMsg.velocityFrame}")
            first_message = False

        print(f"position = {odometryMsg.pose.position.x}, {odometryMsg.pose.position.y}, {odometryMsg.pose.position.z}")
        print(f"orientation = {odometryMsg.pose.orientation.w}, {odometryMsg.pose.orientation.x}, {odometryMsg.pose.orientation.y}, {odometryMsg.pose.orientation.z}")

def main():  

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_odometry_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    # create subscriber and connect callback
    sub = CapnpSubscriber("Odometry3d", "S0/vio_odom")
    sub.set_callback(callback)
    
    # idle main thread
    while ecal_core.ok():
        time.sleep(0.1)
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()