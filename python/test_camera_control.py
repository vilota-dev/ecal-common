#!/usr/bin/env python3

import sys
import time
import keyboard # has to be installed using `sudo pip3 install keyboard`

import capnp

import ecal.core.core as ecal_core
from byte_publisher import BytePublisher

capnp.add_import_hook(['../src/capnp'])

import cameracontrol_capnp as eCALCameraControl

control_msg_cache = eCALCameraControl.CameraControl.new_message()

control_msg = eCALCameraControl.CameraControl.new_message()

pub = None
seq = 0

def reset_control_msg():
    # initialise default values
    control_msg.streaming = -1
    control_msg.exposureUSec = -1
    control_msg.gain = -1
    control_msg.exposureCompensation = -10
    control_msg.sensorIdx = -1

def publish(msg):

    global seq

    msg.header.stamp = time.monotonic_ns()
    msg.header.seq = seq
    seq += 1

    print(msg)
    pub.send(msg.to_bytes())

def handle_up_key(e):

    reset_control_msg()

    if control_msg_cache.exposureUSec <= 32900 and control_msg_cache.gain <= 1590:
        control_msg_cache.exposureUSec += 100
        control_msg_cache.gain += 10

    control_msg.exposureUSec = control_msg_cache.exposureUSec
    control_msg.gain = control_msg_cache.gain

    publish(control_msg)

def handle_down_key(e):

    reset_control_msg()

    if control_msg_cache.exposureUSec >= 200 and control_msg_cache.gain >= 110:
        control_msg_cache.exposureUSec -= 100
        control_msg_cache.gain -= 10

    control_msg.exposureUSec = control_msg_cache.exposureUSec
    control_msg.gain = control_msg_cache.gain

    publish(control_msg)

def handle_left_key(e):

    reset_control_msg()

    if control_msg_cache.exposureCompensation > -9:
        control_msg_cache.exposureCompensation -= 1

    control_msg.exposureCompensation = control_msg_cache.exposureCompensation

    publish(control_msg)

def handle_right_key(e):

    reset_control_msg()

    if control_msg_cache.exposureCompensation < 9:
        control_msg_cache.exposureCompensation += 1

    control_msg.exposureCompensation = control_msg_cache.exposureCompensation

    publish(control_msg)

def handle_space_key(e):

    reset_control_msg()

    if control_msg_cache.streaming:
        control_msg_cache.streaming = 0
    else:
        control_msg_cache.streaming = 1

    control_msg.streaming = control_msg_cache.streaming

    # if control_msg_cache.streaming:
    #     # force manual exposure on restart
    #     control_msg.exposureUSec = control_msg_cache.exposureUSec
    #     control_msg.gain = control_msg_cache.gain

    publish(control_msg)


def main():

    global pub

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_camera_control_pub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    # create publisher

    pub = BytePublisher("S0/camera_control_in")

    # initialise initial setup
    control_msg_cache.exposureUSec = 400
    control_msg_cache.gain = 100
    control_msg_cache.exposureCompensation = 0
    control_msg_cache.streaming = 0

    keyboard.on_press_key("left", handle_left_key)
    keyboard.on_press_key("right", handle_right_key)
    keyboard.on_press_key("up", handle_up_key)
    keyboard.on_press_key("down", handle_down_key)

    keyboard.on_press_key("space", handle_space_key)
    
    # idle main thread
    while ecal_core.ok():
        time.sleep(0.1)
        
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()