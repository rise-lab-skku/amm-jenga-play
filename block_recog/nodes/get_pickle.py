#!/usr/bin python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest
import tf
import geometry_msgs.msg
import numpy as np
import pickle
import cv2
import glob
import open3d as o3d
from matplotlib import pyplot as plt
from func import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def image_callback1(msg):
    # Convert ROS images to OpenCV format
    img_color = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    ## Save pickle
    with open('/home/hr/Desktop/rgb.p', 'wb') as rgb:
        pickle.dump(img_color, rgb)

def image_callback2(msg):
    # Convert ROS images to OpenCV format
    img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    ## Save pickle
    with open('/home/hr/Desktop/dep.p', 'wb') as dep:
        pickle.dump(img_depth, dep)



if __name__ == '__main__':
    rospy.init_node('request_node')  # 노드 초기화

    rospy.Subscriber('/rgb/image_raw', Image, image_callback1)
    rospy.Subscriber('/depth_to_rgb/image_raw', Image, image_callback2)

    rospy.spin()