#!/usr/bin/env python  
import roslib
import rospy
import tf
import geometry_msgs.msg
import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pickle

from func import *

bridge = CvBridge()

def transform_mat_from_trans_rot(trans, rot):
    e1, e2, e3, e4 = rot
    trans_matrix = np.array([[1-2*(e2**2)-2*(e3**2), 2*(e1*e2-e3*e4), 2*(e1*e3+e2*e4), trans[0]],
                                [2*(e1*e2+e3*e4), 1-2*(e1**2)-2*(e3**2), 2*(e2*e3-e1*e4), trans[1]],
                                [2*(e1*e3-e2*e4), 2*(e2*e3+e1*e4), 1-2*(e1**2)-2*(e2**2), trans[2]],
                                [0, 0, 0, 1]])
    
    return trans_matrix

def image_callback1(msg):
    # Convert ROS images to OpenCV format
    img_color = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    ## Save pickle
    with open('rgb.p', 'wb') as rgb:
        pickle.dump(img_color, rgb)
        
def image_callback2(msg):
    # Convert ROS images to OpenCV format
    img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    
    ## Save pickle
    with open('dep.p', 'wb') as dep:
        pickle.dump(img_depth, dep)


if __name__ == '__main__':
    rospy.init_node('block_recognition_node')

    rospy.Subscriber('/rgb/image_raw', Image, image_callback1)
    rospy.Subscriber('/depth_to_rgb/image_raw', Image, image_callback2)


    listener = tf.TransformListener()
    
    img_depth = None
    img_color = None
    

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        if ((img_depth is None) and (img_color is None)):
            with open('dep.p', 'rb') as dep:
                img_depth = pickle.load(dep, encoding="16UC1")
            with open('rgb.p', 'rb') as rgb:
                img_color = pickle.load(rgb)

        try:
            # lookupTransform('target', 'source', Time)
            (trans_cali,rot_cali) = listener.lookupTransform('camera_base', 'panda_hand', rospy.Time(0))
            (trans_world,rot_world) = listener.lookupTransform('world', 'panda_hand', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # rospy.loginfo((trans, rot))
        
        cali_transform_matrix = transform_mat_from_trans_rot(trans_cali, rot_cali)
        to_world_transform_matrix = transform_mat_from_trans_rot(trans_world, rot_cali)
        
        
        
        rate.sleep()
        
        # here return can i use the matrix as function works?