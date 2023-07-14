#!/usr/bin/env python

import numpy as np
import cv2
import glob
import open3d as o3d
from matplotlib import pyplot as plt
from func import *

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pickle


bridge = CvBridge()

def get_rgb_img(msg):
        try:
            rgb_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        with open("/home/shs/Desktop/rgb.p", "wb") as rgb:
            pickle.dump(rgb_img, rgb)
            
def get_depth_img(msg):
        try:
            depth_img = bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            print(e)
            
        with open("/home/shs/Desktop/depth.p", "wb") as depth:
            pickle.dump(depth_img, depth)

transform = None

def image_callback(rgb, dep):
    global transform
    
    if transform is None:
        with open('/home/shs/Desktop/dep.p', 'rb') as dep:
            img_depth = pickle.load(dep, encoding="16UC1")
        
        with open('/home/shs/Desktop/rgb.p', 'rb') as rgb:
            img_color = pickle.load(rgb)
        
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
                
            # exec(f"blocks_pcd_{color} = blocks_pcd")
            # exec(f"blocks_pcd_by_color.append(blocks_pcd_{color})")
            blocks_pcd_by_color.append(blocks_pcd)
                
        pcd_combined = combine_pcd(all_pcd)
        
        mesh_tower = o3d.io.read_triangle_mesh("block_recog/mesh/jenga_tower_side_xy.stl")
        mesh_tower.compute_vertex_normals()

        pcd_target = mesh_tower.sample_points_uniformly(number_of_points=len(pcd_combined.points))
        
        source, target, resize, move = prepare_icp(pcd_combined, pcd_target)
        
        threshold = 10
        transform_matrix = do_ICP(source, target, threshold)
        
        transform = (transform_matrix, resize, move)
    
    target_block_color = None
    target_block_label = None
    
    transform_matrix, resize, move = transform
    
    for col, pcds in zip(colors, blocks_pcd_by_color):
        if col != target_block_color:
            continue
        
        for idx, pcd in enumerate(pcds):
            if idx != target_block_label:
                continue
            
            pcd_new = transform_blocks(pcd, transform_matrix)
            
            box_extent = pcd_new.get_axis_aligned_bounding_box().get_extent()
            
            center_coordinate = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points()).mean(axis=0)
            
            # print("BOX MEAN X,Y and Z Coordinate")
            x_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:,0].mean()
            y_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:,1].mean()
            z_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:,2].mean()
            
            if box_extent[1] > 70:
                print("PULL DIRECTION : X")
                vector = np.array([-1, 0, 0])
                
                cen_x = x_mean #- 25/2
                cen_y = y_mean #- 75/2
                cen_z = z_mean

                target_x = cen_x + 100
                target_y = cen_y
                target_z = cen_z
                
            elif box_extent[0] > 70:
                print("PULL DIRECTION : Y")
                vector = np.array([0, -1, 0])
                
                cen_x = x_mean# - 75/2
                cen_y = y_mean# - 25/2
                cen_z = z_mean

                target_x = cen_x
                target_y = cen_y + 100
                target_z = cen_z
                
            elif abs(center_coordinate[0]) < 10 and box_extent [1] < 15.1:
                print("PUSH DIRECTION : Y or -Y")
                vector = np.array([0, -1, 0])
                
                cen_x = x_mean# - 25/2
                cen_y = y_mean - 75/2
                cen_z = z_mean

                target_x = cen_x
                target_y = cen_y + 150
                target_z = cen_z
                
            elif abs(center_coordinate[1]) < 10 and box_extent [0] < 15.1:
                print("PUSH DIRECTION : X or -X")
                vector = np.array([-1, 0, 0])
                
                cen_x = x_mean - 75/2
                cen_y = y_mean# - 25/2
                cen_z = z_mean

                target_x = cen_x + 150
                target_y = cen_y
                target_z = cen_z
                
            elif box_extent[1] < 15.1:
                print("PULL DIRECTION : -X")
                vector = np.array([1, 0, 0])
                
                cen_x = x_mean# - 25/2
                cen_y = y_mean - 75/2
                cen_z = z_mean


                target_x = cen_x - 100
                target_y = cen_y
                target_z = cen_z

                
            elif box_extent[0] < 15.1:
                print("PULL DIRECTION : -Y")
                vector = np.array([0, 1, 0])

                cen_x = x_mean - 75/2
                cen_y = y_mean# - 25/2
                cen_z = z_mean

                target_x = cen_x
                target_y = cen_y - 100
                target_z = cen_z
                
        
        point = np.array([target_x, target_y, target_z])
        new_point = mesh_coodrd_to_cam_coord(point, transform_matrix, resize, move)
        new_vector = mesh_coodrd_to_cam_coord(vector, transform_matrix, resize, move)

        
        return new_point, new_vector


def main():
    rospy.init_node('image_processing_node')
    rospy.Subscriber('/rgb/image_raw', Image, get_rgb_img)
    rospy.Subscriber('/depth_to_rgb/image_raw', Image, get_depth_img)
    rospy.spin()

if __name__ == '__main__':
    main()
