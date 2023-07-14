#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from your_package.srv import CalculateTransformMatrix
import cv2
import numpy as np

bridge = CvBridge()

def image_callback(rgb_msg, depth_msg):
    # Convert ROS images to OpenCV format
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

    # Perform depth image processing or point cloud extraction
    # ...

    # Request the CalculateTransformMatrix service
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
    rospy.init_node('get_transformmation', anonymous=True)
    rospy.Subscriber("rgb/image_raw", Image, image_callback)
    rospy.Subscriber("depth_to_rgb/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
