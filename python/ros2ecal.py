#!/usr/bin/env python3

import sys

import rospy
from sensor_msgs.msg import CompressedImage

import capnp
import cv2
import numpy as np

import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber

capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage



pub = None
image_size = None

def callback(msg):

    global pub
    global image_size

    if msg.format == "jpeg":
        np_arr = np.frombuffer(msg.data)

        if image_size is None:
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_size = image.shape
            
        
        if pub is not None:
            image_msg = eCALImage.Image.new_message()
            image_msg.header.seq = msg.header.seq
            image_msg.data = msg.data
            image_msg.width = image_size[1]
            image_msg.height = image_size[0]
            image_msg.encoding = "jpeg"
            pub.send(image_msg.to_bytes())
        else:
            print("not sending")
    else:
        raise RuntimeError(f"ros compression format not recognised: {msg.format}")

    # cv2.imshow("image", image)
    # cv2.waitKey(3)


def main():
    rospy.init_node('image2ecal', anonymous=True)

    rospy.Subscriber("/camera360_nodelet_loader/compressed/image_raw", CompressedImage, callback, queue_size=1)


    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "ros2ecal")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    global pub
    pub = BytePublisher("raw_fisheye_image")

    rospy.spin()

    # finalize eCAL API
    ecal_core.finalize()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()