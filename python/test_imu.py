#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

capnp.add_import_hook(['../src/capnp'])

import imulist_capnp as eCALImuList

first_message = True
curr_seq = 0

def callback(type, topic_name, msg, time):

    global first_message
    global curr_seq

    # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
    
    with eCALImuList.ImuList.from_bytes(msg) as imuListMsg:
        if first_message:
            print(f"extrinsic = {imuListMsg.list[0].extrinsic}")
            print(f"time_offset_ns = {imuListMsg.list[0].intrinsic.timeOffsetNs}")
            print(f"update_rate = {imuListMsg.list[0].intrinsic.updateRate}")
            first_message = False

        for imuMsg in imuListMsg.list:
            header = imuMsg.header
            print(f"seq = {header.seq}")
            print(f"latency device = {header.latencyDevice / 1e6} ms")
            print(f"latency host = {header.latencyHost / 1e6} ms")
            accel = np.array([imuMsg.linearAcceleration.x, imuMsg.linearAcceleration.y, imuMsg.linearAcceleration.z])
            gyro = np.array([imuMsg.angularVelocity.x, imuMsg.angularVelocity.y, imuMsg.angularVelocity.z])
            print(f"accel = {accel}")
            print(f"gyro = {gyro}")

def main():  

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_imu_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    # create subscriber and connect callback
    sub = CapnpSubscriber("Imu", "S0/imu")
    sub.set_callback(callback)
    
    # idle main thread
    while ecal_core.ok():
        time.sleep(0.1)
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()
