#!/usr/bin/env python
import numpy as np
import cv2
import glob
import open3d as o3d
from matplotlib import pyplot as plt
from func import *
import pickle

import rospy
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from cv_bridge import CvBridge, CvBridgeError


class Registration:

    def __init__(self):
        
        self.image_tower_color = rospy.Publisher("/jenga/img/tower_color", Image, queue_size=10)
        self.image_tower_mask = rospy.Publisher("/jenga/img/tower_mask", Image, queue_size=10)
        self.trans_matrix = rospy.Publisher('/jenga/transform_matrix', numpy_msg(Floats), queue_size=10)
        self.move = rospy.Publisher('/jenga/move', numpy_msg(Floats), queue_size=10)
        self.resize = rospy.Publisher('/jenga/resize', Floats, queue_size=10)
        
        self.bridge = CvBridge()
        
        self.rgb = None
        self.depth = None

    def get_rgb_img(self, msg):
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        self.rgb = rgb_img
        with open("/home/shs/Desktop/rgb.p", "wb") as rgb:
            pickle.dump(rgb_img, rgb)
        self.get_transform_matrix()
        
        # try:
        #     self.image_pub_green.publish(self.bridge.cv2_to_imgmsg(img_result_green, "bgr8"))
        #     self.image_pub_purple.publish(self.bridge.cv2_to_imgmsg(img_result_purple, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)
        
    def get_depth_img(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            print(e)
            
        self.depth = depth_img
        with open("/home/shs/Desktop/dep.p", "wb") as dep:
            pickle.dump(depth_img, dep)
        self.get_transform_matrix()
        
    def get_transform_matrix(self):
        img_color = self.rgb
        img_depth = self.depth
        
        # height, width = img_color.shape[:2] # 이미지의 높이와 너비 불러옴, 가로 [0], 세로[1]
        # cv2.imwrite("/home/shs/Desktop/rgb.png", img_color)
        # cv2.imwrite("/home/shs/Desktop/dep.png", img_depth)

        img_hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV) # cvtColor 함수를 이용하여 hsv 색공간으로 변환
        
        colors = ['green', 'pink', 'yellow', 'blue', 'violet', 'red']
        
        blocks_rgb_by_color = []
        blocks_mask_by_color = []
        
        for color in colors:
            blocks_color, blocks_mask = img_masking(img_hsv, img_color, color)
            exec(f"blocks_rgb_{color} = blocks_color")
            exec(f"blocks_mask_{color} = blocks_mask")
            exec(f"blocks_rgb_by_color.append(blocks_rgb_{color})")
            exec(f"blocks_mask_by_color.append(blocks_mask_{color})")
            
        tower_mask = 0
        tower_color = 0
        for mask, color in zip(blocks_mask_by_color, blocks_rgb_by_color):
            for block_m in mask:
                tower_mask += block_m
            
            for block_c in color:
                tower_color += block_c
                
        try:
            self.image_tower_color.publish(self.bridge.cv2_to_imgmsg(tower_color, "bgr8"))
            self.image_tower_mask.publish(self.bridge.cv2_to_imgmsg(tower_mask, "mono8"))
        except CvBridgeError as e:
            print(e)
        
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
            
            exec(f"blocks_pcd_{color} = blocks_pcd")
            exec(f"blocks_pcd_by_color.append(blocks_pcd_{color})")
            
        pcd_combined = combine_pcd(all_pcd)
        
        mesh_tower = o3d.io.read_triangle_mesh("/home/shs/catkin_ws/src/block_recog/mesh/jenga_tower_side_xy.stl")
        mesh_tower.compute_vertex_normals()

        pcd_target = mesh_tower.sample_points_uniformly(number_of_points=len(pcd_combined.points))
        
        source, target, resize, move = prepare_icp(pcd_combined, pcd_target)
        
        threshold = 10
        trans_matrix = do_ICP(source, target, threshold)
        
        # PUBLISH TRANS_MATRIX, MOVE, RESIZES
        try:
            self.trans_matrix.publish(trans_matrix)
            self.move.publish(move)
            self.resize.publish(resize)
        except CvBridgeError as e:
            print(e)
        
if __name__ == '__main__':
    rospy.init_node('get_transformmation', anonymous=True)
    
    ic = Registration()
    
    rospy.Subscriber("/depth_to_rgb/image_raw", Image, ic.get_depth_img)
    rospy.Subscriber("/rgb/image_raw", Image, ic.get_rgb_img)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

