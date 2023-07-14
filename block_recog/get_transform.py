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

def calculate_transform_matrix(request):

    # Convert ROS images to OpenCV format
    img_color = bridge.imgmsg_to_cv2(request.rgb_image, desired_encoding='bgr8')
    img_depth = bridge.imgmsg_to_cv2(request.depth_image, desired_encoding='16UC1')

    img_hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV) # cvtColor 함수를 이용하여 hsv 색공간으로 변환
    
    colors = ['green', 'pink', 'yellow', 'blue', 'violet', 'red']
    
    blocks_rgb_by_color = []
    blocks_mask_by_color = []

    for color in colors:
        
        blocks_color, blocks_mask = img_masking(img_hsv, img_color, color)
        
        blocks_rgb_by_color.append(blocks_color)
        blocks_mask_by_color.append(blocks_mask)
        
    tower_mask = 0
    tower_color = 0
    for mask, color in zip(blocks_mask_by_color, blocks_rgb_by_color):
        for block_m in mask:
            tower_mask += block_m
        
        for block_c in color:
            tower_color += block_c
    
    # temp intrinsic matrix
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.intrinsic_matrix = [[968.813, 0, 1023.83],
                              [0, 968.635, 775.975],
                              [0, 0, 1]]
    
    masked_depth = cv2.bitwise_and(img_depth, img_depth, mask = tower_mask)
    tower_pcd = get_pointcloud_from_color_depth(color_image=tower_color, depth_image=masked_depth, intrinsic=intrinsic)
    
    blocks_pcd_by_color = []
    all_pcd = []
    for color, block_mask in zip(colors, blocks_mask_by_color):
        # print(color)
        blocks_pcd = []
        for msk in block_mask:
            masked_block_rgb = cv2.bitwise_and(tower_color, tower_color, mask = msk)
            masked_block_depth = cv2.bitwise_and(img_depth, img_depth, mask = msk)
            
            # Get Each Block's PointCloud
            pcd = get_pointcloud_from_color_depth(color_image=masked_block_rgb, depth_image=masked_block_depth, intrinsic=intrinsic)
            
            # Remove Outlier Points
            pcd, _ = pcd.remove_radius_outlier(1024, 0.025)
            blocks_pcd.append(pcd)
            all_pcd.append(pcd)
            
    pcd_combined = combine_pcd(all_pcd)
    
    mesh_tower = o3d.io.read_triangle_mesh("block_recog/mesh/jenga_tower_side_xy.stl")
    mesh_tower.compute_vertex_normals()

    pcd_target = mesh_tower.sample_points_uniformly(number_of_points=len(pcd_combined.points))
    
    source, target, resize, move = prepare_icp(pcd_combined, pcd_target)
    
    threshold = 10
    transform_matrix = do_ICP(source, target, threshold)

    # Return the transform matrix
    return transform_matrix, resize, move
