#!/usr/bin/env python3

import sys
import time
import threading

import capnp
import numpy as np

import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber


capnp.add_import_hook(['../src/capnp'])

import rospy
from nav_msgs.msg import Odometry

import odometry3d_capnp as eCALOdometry3d



class RosOdometryPublisher:

    def __init__(self, topic) -> None:
        self.first_message = True
        self.ros_pub = rospy.Publisher(topic, Odometry, queue_size=10)
        

    def callback(self, topic_name, msg, time):

        # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
        
        with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:
            print(f"seq = {odometryMsg.header.seq}")
            print(f"latency device = {odometryMsg.header.latencyDevice / 1e6} ms")
            print(f"latency host = {odometryMsg.header.latencyHost / 1e6} ms")

            if self.first_message:
                print(f"bodyFrame = {odometryMsg.bodyFrame}")
                print(f"referenceFrame = {odometryMsg.referenceFrame}")
                print(f"velocityFrame = {odometryMsg.velocityFrame}")
                self.first_message = False

            print(f"position = {odometryMsg.pose.position.x}, {odometryMsg.pose.position.y}, {odometryMsg.pose.position.z}")
            print(f"orientation = {odometryMsg.pose.orientation.w}, {odometryMsg.pose.orientation.x}, {odometryMsg.pose.orientation.y}, {odometryMsg.pose.orientation.z}")

            ros_msg = Odometry();
            ros_msg.header.seq = odometryMsg.header.seq
            ros_msg.header.stamp.from_sec(odometryMsg.header.stamp / 1.0e9)
            ros_msg.header.frame_id = "map"

            ros_msg.pose.pose.position.x = odometryMsg.pose.position.x
            ros_msg.pose.pose.position.y = odometryMsg.pose.position.y
            ros_msg.pose.pose.position.z = odometryMsg.pose.position.z

            ros_msg.pose.pose.orientation.w = odometryMsg.pose.orientation.w
            ros_msg.pose.pose.orientation.x = odometryMsg.pose.orientation.x
            ros_msg.pose.pose.orientation.y = odometryMsg.pose.orientation.y
            ros_msg.pose.pose.orientation.z = odometryMsg.pose.orientation.z

            self.ros_pub.publish(ros_msg)            


def main():  

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_odometry_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    rospy.init_node("ros_odometry_publisher")

    

    topic = "S0/vio_odom"

    ros_odometry_pub = RosOdometryPublisher(topic)

    # create subscriber and connect callback
    sub = ByteSubscriber(topic)
    sub.set_callback(ros_odometry_pub.callback)
    
    # idle main thread
    # while ecal_core.ok():
    #     time.sleep(0.1)
    rospy.spin()
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()