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

def image_callback(rgb_msg, depth_msg, transform_matrix, resize, move):
    # Convert ROS images to OpenCV format
    img_color = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
    img_depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

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
    intrinsic.intrinsic_matrix = [[971.179, 0, 1025.07],[0, 970.984, 778.291],[0, 0, 1]]
    
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
            pcd, _ = pcd.remove_radius_outlier(512, 0.0001)
            blocks_pcd.append(pcd)
            all_pcd.append(pcd)
            
        # exec(f"blocks_pcd_{color} = blocks_pcd")
        # exec(f"blocks_pcd_by_color.append(blocks_pcd_{color})")
        blocks_pcd_by_color.append(blocks_pcd)
            
    pcd_combined = combine_pcd(all_pcd)
    
    target_block_color = None
    target_block_label = None
    
    for col, pcds in zip(colors, blocks_pcd_by_color):
        if col != target_block_color:
            continue
        print(col)
        
        for idx, pcd in enumerate(pcds):
            if idx != target_block_label:
                continue
            
            # print("--------------------------------")
            # print(idx)
            pcd_new = transform_blocks(pcd, transform_matrix, resize, move)
            
            # o3d.visualization.draw_geometries([pcd_new, mesh_frame, pcd_new.get_axis_aligned_bounding_box()])
            
            box_extent = pcd_new.get_axis_aligned_bounding_box().get_extent()
            # print("BOX EXTENT : ", box_extent)
            
            center_coordinate = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points()).mean(axis=0)
            
            # print(np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points()))
            # print("BOX CENTER COORDINATE : ", center_coordinate)
            
            # print("BOX MAX X,Y and MEAN Z Coordinate")
            x_max = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:,0].max()
            y_max = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:,1].max()
            z_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:,2].mean()
            
            if box_extent[1] > 70:
                # print("PULL DIRECTION : X")
                vector = np.array([-1, 0, 0])

                cen_x = x_max - 25/2
                cen_y = y_max - 75/2
                cen_z = z_mean

                target_x = cen_x + 100
                target_y = cen_y
                target_z = cen_z
                
            elif box_extent[0] > 70:
                # print("PULL DIRECTION : Y")
                vector = np.array([0, -1, 0])

                cen_x = x_max - 75/2
                cen_y = y_max - 25/2
                cen_z = z_mean

                target_x = cen_x
                target_y = cen_y + 100
                target_z = cen_z
                
            elif abs(center_coordinate[0]) < 10 and box_extent [1] < 20:
                # print("PUSH DIRECTION : Y or -Y")
                vector = np.array([0, -1, 0])

                cen_x = x_max - 25/2
                cen_y = y_max - 75/2
                cen_z = z_mean

                target_x = cen_x
                target_y = cen_y + 150
                target_z = cen_z
                
            elif abs(center_coordinate[1]) < 10 and box_extent [0] < 20:
                # print("PUSH DIRECTION : X or -X")
                vector = np.array([-1, 0, 0])

                cen_x = x_max - 75/2
                cen_y = y_max - 25/2
                cen_z = z_mean

                target_x = cen_x + 150
                target_y = cen_y
                target_z = cen_z
                
            elif box_extent[1] < 20:
                # print("PULL DIRECTION : -X")
                vector = np.array([1, 0, 0])

                cen_x = x_max - 25/2
                cen_y = y_max - 75/2
                cen_z = z_mean

                target_x = cen_x - 100
                target_y = cen_y
                target_z = cen_z
                
            elif box_extent[0] < 20:
                # print("PULL DIRECTION : -Y")
                vector = np.array([0, 1, 0])

                cen_x = x_max - 75/2
                cen_y = y_max - 25/2
                cen_z = z_mean

                target_x = cen_x
                target_y = cen_y - 100
                target_z = cen_z
    
    return (target_x, target_y, target_z), vector


def main():
    rospy.init_node('image_processing_node')
    rospy.Subscriber('/rgb/image_raw', Image, image_callback)
    rospy.Subscriber('/depth_to_rgb/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
