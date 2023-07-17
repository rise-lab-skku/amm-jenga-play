#!/usr/bin/env python

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
    with open('rgb.p', 'wb') as rgb:
        pickle.dump(img_color, rgb)
        
def image_callback2(msg):
    # Convert ROS images to OpenCV format
    img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    
    ## Save pickle
    with open('dep.p', 'wb') as dep:
        pickle.dump(img_depth, dep)


def request_transform_matrix():
    rospy.wait_for_service('transform_service')  # 서비스가 등록될 때까지 대기

    try:
        transform_service = rospy.ServiceProxy('transform_service', Trigger)  # 서비스 프록시 생성
        request = TriggerRequest()  # 서비스 요청 메시지 생성

        response = transform_service(request)  # 서비스 호출

        if response.success:
            transform_matrix, blocks_pcd_by_color = response.message  # 변환 행렬을 변수에 저장
            rospy.loginfo('Transform matrix received: %s', transform_matrix)
        else:
            rospy.logwarn('Failed to receive transform matrix: %s', response.message)
    
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: %s', str(e))
        
    return transform_matrix, blocks_pcd_by_color
        
def transform_mat_from_trans_rot(trans, rot):
    e1, e2, e3, e4 = rot
    trans_matrix = np.array([[1-2*(e2**2)-2*(e3**2), 2*(e1*e2-e3*e4), 2*(e1*e3+e2*e4), trans[0]],
                                [2*(e1*e2+e3*e4), 1-2*(e1**2)-2*(e3**2), 2*(e2*e3-e1*e4), trans[1]],
                                [2*(e1*e3-e2*e4), 2*(e2*e3+e1*e4), 1-2*(e1**2)-2*(e2**2), trans[2]],
                                [0, 0, 0, 1]])
    
    return trans_matrix

if __name__ == '__main__':
    rospy.init_node('request_node')  # 노드 초기화
    
    rospy.Subscriber('/rgb/image_raw', Image, image_callback1)
    rospy.Subscriber('/depth_to_rgb/image_raw', Image, image_callback2)
    
    transform_matrix, blocks_pcd_by_color = request_transform_matrix()  # 서비스 요청
    
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # lookupTransform('target', 'source', Time)
            (trans_cali,rot_cali) = listener.lookupTransform('camera_base', 'panda_hand', rospy.Time(0))
            # (trans_world,rot_world) = listener.lookupTransform('panda_link0', 'panda_hand', rospy.Time(0))    # world
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # rospy.loginfo((trans, rot))
        
        # TARGET BLOCK INPUT (EX. 'green 1')
        target_block = "red 1"
        center_coordinate, target_coordinate, vector = get_coordinate(target_block, blocks_pcd_by_color, transform_matrix)
        
        jenga_first_coord1 = np.array([37.5, 37.5, 0])
        jenga_first_coord2 = np.array([37.5, -37.5, 0])
        jenga_first_coord3 = np.array([-37.5, -37.5, 0])
        jenga_first_coord4 = np.array([-37.5, 37.5, 0])
        
        jenga_first_coord = [jenga_first_coord1, jenga_first_coord2, jenga_first_coord3, jenga_first_coord4]
        
        
        
        cali_transform_matrix = transform_mat_from_trans_rot(trans_cali, rot_cali)
        # to_world_transform_matrix = transform_mat_from_trans_rot(trans_world, rot_cali)
        
        mesh_to_camera_matrix = np.linalg.inv(transform_matrix)
        camera_to_hand_matrix = np.linalg.inv(cali_transform_matrix)
        
        jenga_transformed_coord = []
        
        for jfc in jenga_first_coord:
            for tran_mat in [mesh_to_camera_matrix, camera_to_hand_matrix]:
                jenga_transformed_coord.append(jenga_transformed_coord)
        
        rospy.loginfo("jenga_transformed_coord")        
        rospy.loginfo(jenga_transformed_coord)
        
        for tran_mat in [mesh_to_camera_matrix, camera_to_hand_matrix]:
            center_coordinate = coordinate_transform(center_coordinate, mesh_to_camera_matrix)
            target_coordinate = coordinate_transform(target_coordinate, mesh_to_camera_matrix)
        
        
        
        vector = center_coordinate - target_coordinate
        
        rospy.loginfo("center_coordinate")
        rospy.loginfo(center_coordinate)
        rospy.loginfo("target_coordinate")
        rospy.loginfo(target_coordinate)
        rospy.loginfo("vector")
        rospy.loginfo(vector)
            