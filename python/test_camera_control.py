#!/usr/bin/env python3

import sys
import time

import capnp
import numpy as np
import cv2

import ecal.core.core as ecal_core
from byte_publisher import BytePublisher

capnp.add_import_hook(['../src/capnp'])

import cameracontrol_capnp as eCALCameraControl


def main():  

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_camera_control_pub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    # create publisher

    pub = BytePublisher("S0/camera_control_in")

    control_msg = eCALCameraControl.CameraControl.new_message()

    exposure = 4000
    iso = 100

    step_exposure = 100
    step_iso = 10
    
    # idle main thread
    while ecal_core.ok():
        
        control_msg.exposureUSec = exposure
        control_msg.gain = iso

        pub.send(control_msg.to_bytes())

        print(f"sent exposure = {exposure}, iso = {iso}")
        time.sleep(1)

        exposure += step_exposure
        iso += step_iso
        
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()