#!/usr/bin/env python

import numpy as np
import cv2
import glob
import open3d as o3d
from matplotlib import pyplot as plt
from func import *

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pickle

bridge = CvBridge()

def image_callback1(msg):
    # Convert ROS images to OpenCV format
    img_color = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    
    ## Save pickle
    with open('rgb.p', 'wb') as rgb:
        pickle.dump(img_color, rgb)
        
def image_callback2(msg):
    # Convert ROS images to OpenCV format
    img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    
    ## Save pickle
    with open('rgb.p', 'wb') as rgb:
        pickle.dump(img_depth, rgb)


def main():
    rospy.init_node('image_processing_node')
    rospy.Subscriber('/rgb/image_raw', Image, image_callback1)
    rospy.Subscriber('/depth_to_rgb/image_raw', Image, image_callback2)
    rospy.spin()

if __name__ == '__main__':
    main()
