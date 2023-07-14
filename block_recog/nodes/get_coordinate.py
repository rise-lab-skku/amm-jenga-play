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
from block_recog.srv import CalculateTransformMatrix


bridge = CvBridge()

def image_callback(rgb_msg, depth_msg):
    # Convert ROS images to OpenCV format
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

    
    rospy.wait_for_service('calculate_transform_matrix')
    try:
        calculate_transform_matrix = rospy.ServiceProxy('calculate_transform_matrix', CalculateTransformMatrix)
        response = calculate_transform_matrix(rgb_image, depth_image)

        # Process the response (e.g., access the transform matrix)
        transform_matrix = response.transform_matrix
        # ...
        

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('image_processing_node')
    rospy.Subscriber('/rgb/image_raw', Image, image_callback)
    rospy.Subscriber('/depth_to_rgb/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
