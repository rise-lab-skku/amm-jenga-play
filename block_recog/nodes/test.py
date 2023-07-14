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

bridge = CvBridge()

def image_callback(rgb_msg, depth_msg):
    # Convert ROS images to OpenCV format
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
    
    cv2.imwrite(rgb_image, "/home/shs/Desktop/rgb.png")
    cv2.imwrite(rgb_image, "/home/shs/Desktop/dep.png")
    
    rospy.wait_for_service('calculate_transform_matrix')
    try:
        calculate_transform_matrix = rospy.ServiceProxy('calculate_transform_matrix', CalculateTransformMatrix)
        response = calculate_transform_matrix(rgb_image, depth_image)

        # Process the response (e.g., access the transform matrix)
        transform_matrix = response.transform_matrix
        
        img_color = rgb_image
        img_depth = depth_image
        
        img_hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV) # cvtColor 함수를 이용하여 hsv 색공간으로 변환
        
        colors = ['green', 'pink', 'yellow', 'blue', 'violet', 'red']
        
        blocks_rgb_by_color = []
        blocks_mask_by_color = []
        
        
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
