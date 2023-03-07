#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np

import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber

capnp.add_import_hook(['../src/capnp'])

import imu_capnp as eCALImu

first_message = True

def callback(topic_name, msg, time):

    global first_message

    # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
    
    with eCALImu.Imu.from_bytes(msg) as imuMsg:
        print(f"seq = {imuMsg.header.seq}")
        print(f"latency device = {imuMsg.header.latencyDevice / 1e6} ms")
        print(f"latency host = {imuMsg.header.latencyHost / 1e6} ms")
        accel = np.array([imuMsg.linearAcceleration.x, imuMsg.linearAcceleration.y, imuMsg.linearAcceleration.z])
        gyro = np.array([imuMsg.angularVelocity.x, imuMsg.angularVelocity.y, imuMsg.angularVelocity.z])
        print(f"accel = {accel}")
        print(f"gyro = {gyro}")

        if first_message:
            print(f"extrinsic = {imuMsg.extrinsic}")
            print(f"time_offset_ns = {imuMsg.intrinsic.timeOffsetNs}")
            print(f"update_rate = {imuMsg.intrinsic.updateRate}")
            first_message = False

def main():  

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_imu_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    # create subscriber and connect callback
    sub = ByteSubscriber("S0/imu")
    sub.set_callback(callback)
    
    # idle main thread
    while ecal_core.ok():
        time.sleep(0.1)
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()