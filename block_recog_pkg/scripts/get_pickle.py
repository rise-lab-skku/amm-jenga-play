#!/usr/bin/python3

import rospy
import pickle

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()


def image_callback_rgb(msg):
    img_color = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Save pickle
    with open("/home/hr/Desktop/dice.p", "wb") as rgb:
        pickle.dump(img_color, rgb)


def image_callback_depth(msg):
    img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    # Save pickle
    with open("/home/hr/Desktop/dep.p", "wb") as dep:
        pickle.dump(img_depth, dep)


if __name__ == "__main__":
    rospy.init_node("request_node")
    rospy.Subscriber("/rgb/image_raw", Image, image_callback_rgb)
    # rospy.Subscriber("/depth_to_rgb/image_raw", Image, image_callback_depth)
    rospy.spin()
