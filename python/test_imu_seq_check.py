#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np

import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber

capnp.add_import_hook(['../src/capnp'])

import imu_capnp as eCALImu

last_seq = None

def callback(topic_name, msg, time):

    global last_seq

    # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
    
    with eCALImu.Imu.from_bytes(msg) as imuMsg:
        print(f"seq = {imuMsg.header.seq}")

        if last_seq is None:
            pass
        else:
            expected_seq = last_seq + imuMsg.seqIncrement
            if imuMsg.header.seq != expected_seq:
                print(f"actual seq {imuMsg.header.seq} and expected {expected_seq} mismatch, last seq {last_seq}")

        last_seq = imuMsg.header.seq

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