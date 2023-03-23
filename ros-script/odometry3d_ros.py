#!/usr/bin/env python3

import sys
import time
import threading
import argparse

import capnp
import numpy as np

import ecal.core.core as ecal_core

import pathlib

current_path = str(pathlib.Path(__file__).parent.resolve())

print("working in path " + current_path)

capnp.add_import_hook([current_path + '/../src/capnp', current_path + '/ecal-common/src/capnp'])

import odometry3d_capnp as eCALOdometry3d

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

import tf


from ecal.core.subscriber import MessageSubscriber

class ByteSubscriber(MessageSubscriber):
  """Specialized publisher subscribes to raw bytes
  """
  def __init__(self, name):
    topic_type = "base:byte"
    super(ByteSubscriber, self).__init__(name, topic_type)
    self.callback = None

  def receive(self, timeout=0):
    """ receive subscriber content with timeout

    :param timeout: receive timeout in ms

    """
    ret, msg, time = self.c_subscriber.receive(timeout)
    return ret, msg, time

  def set_callback(self, callback):
    """ set callback function for incoming messages

    :param callback: python callback function (f(topic_name, msg, time))

    """
    self.callback = callback
    self.c_subscriber.set_callback(self._on_receive)

  def rem_callback(self, callback):
    """ remove callback function for incoming messages

    :param callback: python callback function (f(topic_name, msg, time))

    """
    self.c_subscriber.rem_callback(self._on_receive)
    self.callback = None

  def _on_receive(self, topic_name, msg, time):
    self.callback(topic_name, msg, time)    


class RosOdometryPublisher:

    def publish_tf(self, tf_msg):
        if not self.no_tf_publisher:
                self.static_broadcaster.sendTransform(tf_msg)

    def __init__(self, ros_tf_prefix : str, topic : str, use_monotonic : bool, no_tf_publisher : bool) -> None:
        self.first_message = True
        self.ros_odom_pub = rospy.Publisher(topic, Odometry, queue_size=10)
        self.use_monotonic = use_monotonic
        self.no_tf_publisher = no_tf_publisher
        self.ros_tf_prefix = "/" + ros_tf_prefix + "/"

        print(f"ecal-ros bridge using monotonic = {use_monotonic}")
        print(f"ecal-ros bridge publishing tf = {not no_tf_publisher}")
        print(f"ecal-ros bridge publish topic = {topic}, with tf prefix {self.ros_tf_prefix}")

        # static transforms
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster = tf2_ros.TransformBroadcaster()

        if topic.endswith("_ned"):
            self.isNED = True

            tf_msg = TransformStamped()
            if self.use_monotonic:
                tf_msg.header.stamp = rospy.Time.from_sec(time.monotonic())
            else:
                tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.frame_id = self.ros_tf_prefix + "odom"
            tf_msg.child_frame_id = self.ros_tf_prefix + "odom_ned"

            tf_msg.transform.translation.x = 0
            tf_msg.transform.translation.y = 0
            tf_msg.transform.translation.z = 0

            # R_ned_nwu = np.array ([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            T_nwu_ned = np.identity(4)
            R_nwu_ned = np.array ([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            T_nwu_ned[:3, :3] = R_nwu_ned
            quat = tf.transformations.quaternion_from_matrix(T_nwu_ned)

            tf_msg.transform.rotation.x = quat[0]
            tf_msg.transform.rotation.y = quat[1]
            tf_msg.transform.rotation.z = quat[2]
            tf_msg.transform.rotation.w = quat[3]

            time.sleep(0.5)

            self.publish_tf(tf_msg)

            self.tf_msg_odom = tf_msg

            time.sleep(0.1)


            tf_msg.header.frame_id = self.ros_tf_prefix + "base_link"
            tf_msg.child_frame_id = self.ros_tf_prefix + "base_link_frd"

            self.publish_tf(tf_msg)

            self.tf_msg_base_link = tf_msg


        else:
            self.isNED = False
        

    def callback(self, topic_name, msg, time_ecal):

        # need to remove the .decode() function within the Python API of ecal.core.subscriber ByteSubscriber
        
        with eCALOdometry3d.Odometry3d.from_bytes(msg) as odometryMsg:

            if self.first_message:
                print(f"bodyFrame = {odometryMsg.bodyFrame}")
                print(f"referenceFrame = {odometryMsg.referenceFrame}")
                print(f"velocityFrame = {odometryMsg.velocityFrame}")
                self.first_message = False

            if odometryMsg.header.seq % 100 == 0:
                print(f"seq = {odometryMsg.header.seq}")
                print(f"latency device = {odometryMsg.header.latencyDevice / 1e6} ms")
                print(f"latency host = {odometryMsg.header.latencyHost / 1e6} ms")
                print(f"position = {odometryMsg.pose.position.x}, {odometryMsg.pose.position.y}, {odometryMsg.pose.position.z}")
                print(f"orientation = {odometryMsg.pose.orientation.w}, {odometryMsg.pose.orientation.x}, {odometryMsg.pose.orientation.y}, {odometryMsg.pose.orientation.z}")
                

                if self.use_monotonic:
                    self.tf_msg_odom.header.stamp = rospy.Time.from_sec(time.monotonic())
                    self.tf_msg_base_link.header.stamp = rospy.Time.from_sec(time.monotonic())
                else:
                    self.tf_msg_odom.header.stamp = rospy.Time.now()
                    self.tf_msg_base_link.header.stamp = rospy.Time.now()

                self.publish_tf(self.tf_msg_odom)
                self.publish_tf(self.tf_msg_base_link)

            ros_msg = Odometry();
            ros_msg.header.seq = odometryMsg.header.seq

            if self.use_monotonic:
                ros_msg.header.stamp = rospy.Time.from_sec(odometryMsg.header.stamp / 1.0e9)
            else:
                ros_msg.header.stamp = rospy.Time.now() #.from_sec(odometryMsg.header.stamp / 1.0e9)

            if self.isNED:
                ros_msg.header.frame_id = self.ros_tf_prefix + "odom_ned"
                ros_msg.child_frame_id = self.ros_tf_prefix + "base_link_frd"
            else:
                ros_msg.header.frame_id = self.ros_tf_prefix + "odom"
                ros_msg.child_frame_id = self.ros_tf_prefix + "base_link"

            ros_msg.pose.pose.position.x = odometryMsg.pose.position.x
            ros_msg.pose.pose.position.y = odometryMsg.pose.position.y
            ros_msg.pose.pose.position.z = odometryMsg.pose.position.z

            ros_msg.pose.pose.orientation.w = odometryMsg.pose.orientation.w
            ros_msg.pose.pose.orientation.x = odometryMsg.pose.orientation.x
            ros_msg.pose.pose.orientation.y = odometryMsg.pose.orientation.y
            ros_msg.pose.pose.orientation.z = odometryMsg.pose.orientation.z

            self.ros_odom_pub.publish(ros_msg)

            # publish

            tf_msg = TransformStamped()
            tf_msg.header.stamp = ros_msg.header.stamp

            if self.isNED:
                tf_msg.header.frame_id = self.ros_tf_prefix + "odom_ned"
                tf_msg.child_frame_id = self.ros_tf_prefix + "base_link_frd"
            else:
                tf_msg.header.frame_id = self.ros_tf_prefix + "odom"
                tf_msg.child_frame_id = self.ros_tf_prefix + "base_link"

            tf_msg.transform.translation.x = odometryMsg.pose.position.x
            tf_msg.transform.translation.y = odometryMsg.pose.position.y
            tf_msg.transform.translation.z = odometryMsg.pose.position.z

            tf_msg.transform.rotation = ros_msg.pose.pose.orientation

            self.publish_tf(tf_msg)


def main():  

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))

    topic_ecal = "S0/vio_odom"
    topic_ros = "/basalt/odom_ned"

    parser = argparse.ArgumentParser()
    parser.add_argument('ecal_topic_in', nargs='?', help="topic of ecal", default=topic_ecal)
    parser.add_argument('ros_topic_out', nargs='?', help="topic of ros", default=topic_ros)
    parser.add_argument('--ros_tf_prefix', type=str, default='S0')
    parser.add_argument('--monotonic_time', action="store_true")
    parser.add_argument('--no_tf_publisher', action="store_true")
    args = parser.parse_known_args()[0]

    args.ros_topic_out = "/" + args.ros_tf_prefix + args.ros_topic_out
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_odometry_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    rospy.init_node("ros_odometry_publisher")

    ros_odometry_pub = RosOdometryPublisher(args.ros_tf_prefix, args.ros_topic_out, args.monotonic_time, args.no_tf_publisher)

    # create subscriber and connect callback
    print(f"ecal-ros bridge subscribe topic: {args.ecal_topic_in}")
    sub = ByteSubscriber(args.ecal_topic_in)
    sub.set_callback(ros_odometry_pub.callback)
    
    # idle main thread
    # while ecal_core.ok():
    #     time.sleep(0.1)
    rospy.spin()
    
    # finalize eCAL API
    ecal_core.finalize()

if __name__ == "__main__":
    main()