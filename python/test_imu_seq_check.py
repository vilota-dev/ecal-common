#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber 

capnp.add_import_hook(['../src/capnp'])

import imulist_capnp as eCALImuList

last_seq = None
last_ecal_time = None

def callback(type, topic_name, msg, time_ecal):

    global last_seq
    global last_ecal_time

    time_now = time.time_ns() / 1e3

    

    # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
    
    with eCALImuList.ImuList.from_bytes(msg) as imuListMsg:
            
        force_print = False
        for imuMsg in imuListMsg.list:
            if last_seq is None:
                pass
            else:
                expected_seq = last_seq + imuMsg.seqIncrement
                if imuMsg.header.seq != expected_seq:
                    print(f"WARN: actual seq {imuMsg.header.seq} and expected {expected_seq} mismatch, last seq {last_seq}")
                    force_print = True

            if force_print or imuMsg.header.seq % 100 == 0:
                decoded_time = time.time_ns() / 1e3
                print(f"seq = {imuMsg.header.seq}, lapse {(time_now - time_ecal)} us, decoding {decoded_time - time_now} us")
                print(f"time interval since the last ecal message at sender {time_ecal - last_ecal_time} us")

            last_seq = imuMsg.header.seq
            last_ecal_time = time_ecal

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
