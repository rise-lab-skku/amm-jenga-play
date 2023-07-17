#!/usr/bin/env python

import rospy
from block_recog.srv import GetWorldCoord, GetWorldCoordRequest, GetWorldCoordResponse, CaptureImage, CaptureImageRequest, CaptureImageResponse
import tf
import geometry_msgs.msg
import numpy as np
import pickle
import time
import cv2
import glob
import open3d as o3d
from matplotlib import pyplot as plt
from func import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

colors = colors = ['green', 'pink', 'yellow', 'blue', 'violet', 'red']


def transform_coordinates(coordinate1, coordinate2, transform_matrix_lst):
    for trans_mat in transform_matrix_lst:
        coordinate1 = coordinate_transform(coordinate1, trans_mat)
        
    for trans_mat in transform_matrix_lst:
        coordinate2 = coordinate_transform(coordinate2, trans_mat)
    
    return coordinate1, coordinate2



class CoordinateServer():
    def __init__(self):
        self.mesh_tower = o3d.io.read_triangle_mesh("/home/shs/catkin_ws/src/block_recog/mesh/jenga_tower_side_xy.stl")
        self.mesh_tower.compute_vertex_normals()
        
        jenga_first_coord1 = np.array([37.5, 37.5, 0])
        jenga_first_coord2 = np.array([37.5, -37.5, 0])
        jenga_first_coord3 = np.array([-37.5, -37.5, 0])
        jenga_first_coord4 = np.array([-37.5, 37.5, 0])
        
        self.jenga_first_coord = [jenga_first_coord1, jenga_first_coord2, jenga_first_coord3, jenga_first_coord4]
        
        
        self.tower_transform_initialized = False
        #-------------------------------------------------------------------------
        
        # with open('/home/shs/catkin_ws/src/block_recog/img/dep.p', 'rb') as dep:
        #     img_depth = pickle.load(dep, encoding="16UC1")
            
        # with open('/home/shs/catkin_ws/src/block_recog/img/rgb.p', 'rb') as rgb:
        #     img_color = pickle.load(rgb)
        
        self.img_depth = None
        self.img_color = None
        
        self.ready_to_capture_image = False
        self.capture_once = 0
        
        rospy.Subscriber('/rgb/image_raw', Image, self.image_callback1)
        rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.image_callback2)
        
        
        capture_service = rospy.Service("CaptureImage", CaptureImage, self.capture_flag)
        
        
        service = rospy.Service('GetWorldCoordinates', GetWorldCoord, self.GetWorldCoordinates)
    
    def find_initial_tower_transform(self):
        
        colors = ['green', 'pink', 'yellow', 'blue', 'violet', 'red']
        blocks_rgb_by_color = []
        blocks_mask_by_color = []
        
        for color in colors:
            blocks_color, blocks_mask = img_masking(self.img_color, color)
            blocks_rgb_by_color.append(blocks_color)
            blocks_mask_by_color.append(blocks_mask)
        
        #--------------------------------------------------------------------------
        
        tower_mask, tower_color = self.get_tower_mask(blocks_mask_by_color, blocks_rgb_by_color)
        # masked_depth = cv2.bitwise_and(img_depth, img_depth, mask = tower_mask)
        # tower_pcd = get_pointcloud_from_color_depth(color_image=tower_color, depth_image=masked_depth, intrinsic=intrinsic)
                
        #--------------------------------------------------------------------------
        pcd_combined, self.block_pcd_by_color = self.build_clean_tower_pcd_from_blocks(blocks_mask_by_color, tower_color, self.img_depth)
        
        #--------------------------------------------------------------------------
        
        self.camera_to_mesh_matrix, mesh_to_camera_matrix = self.transform_matrix_mesh_to_camera(pcd_combined)
        
        #--------------------------------------------------------------------------
        
        listener = tf.TransformListener()

        try:
            # lookupTransform('target', 'source', Time)
            (trans_cam_to_world,rot_cam_to_world) = listener.lookupTransform('world', 'rgb_camera_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            raise 
        
        self.cam_to_world_transform_matrix = transform_mat_from_trans_rot(trans_cam_to_world, rot_cam_to_world)
        
        self.transform_matrix_lst = [mesh_to_camera_matrix, self.cam_to_world_transform_matrix]
        
        self.tower_transform_initialized = True
    
    def capture_flag(self, request):
        rospy.loginfo(f"Service CaptureImage")
        
        resp = CaptureImageResponse()
        
        if self.capture_once > 0:
            resp.status = resp.SKIPPED
            return resp
        
        self.ready_to_capture_image = True
        is_success = self.wait_image(time_threshold=10)
        
        if is_success:
            self.find_initial_tower_transform()
            resp.status = resp.SUCCESS
            self.capture_once += 1
            return resp
        
        resp.status = resp.FAILED
        return resp
    
    def image_callback1(self, msg):
        # Convert ROS images to OpenCV format
        if self.img_color is None and self.ready_to_capture_image:
            self.img_color = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        ## Save pickle
        # with open('/home/shs/catkin_ws/src/block_recog/img/rgb.p', 'wb') as rgb:
        #     pickle.dump(img_color, rgb)
            
    def image_callback2(self, msg):
        # Convert ROS images to OpenCV format
        if self.img_depth is None and self.ready_to_capture_image:
            self.img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        
        ## Save pickle
        # with open('/home/shs/catkin_ws/src/block_recog/img/dep.p', 'wb') as dep:
        #     pickle.dump(img_depth, dep) 
        
    def get_tower_mask(self, blocks_mask_by_color, blocks_rgb_by_color):
        tower_mask = 0
        tower_color = 0
        for mask, color in zip(blocks_mask_by_color, blocks_rgb_by_color):
            for block_m in mask:
                tower_mask += block_m
            
            for block_c in color:
                tower_color += block_c
                
        return tower_mask, tower_color
    
    def build_clean_tower_pcd_from_blocks(self, blocks_mask_by_color, tower_color, img_depth):
        intrinsic = o3d.camera.PinholeCameraIntrinsic()
        intrinsic.intrinsic_matrix = [[968.813, 0, 1023.83],
                                    [0, 968.635, 775.975],
                                    [0, 0, 1]]
        
        blocks_pcd_by_color = []
        all_pcd = []
        for color, block_mask in zip(colors, blocks_mask_by_color):
            blocks_pcd = []
            for msk in block_mask:
                masked_block_rgb = cv2.bitwise_and(tower_color, tower_color, mask = msk)
                masked_block_depth = cv2.bitwise_and(img_depth, img_depth, mask = msk)
                
                # Get Each Block's PointCloud
                pcd = get_pointcloud_from_color_depth(color_image=masked_block_rgb, depth_image=masked_block_depth, intrinsic=intrinsic)
                
                # Remove Outlier Points
                pcd, _ = pcd.remove_radius_outlier(1024, 25)
                blocks_pcd.append(pcd)
                all_pcd.append(pcd)
            
            blocks_pcd_by_color.append(blocks_pcd)
        
        pcd_combined = combine_pcd(all_pcd)
        
        return pcd_combined, blocks_pcd_by_color
    
    def transform_matrix_mesh_to_camera(self, pcd_combined):
        # mesh_tower = o3d.io.read_triangle_mesh("/home/shs/catkin_ws/src/block_recog/mesh/jenga_tower_side_xy.stl")
        # mesh_tower.compute_vertex_normals()
        
        mesh_tower = self.mesh_tower

        pcd_target = mesh_tower.sample_points_uniformly(number_of_points=len(pcd_combined.points))
        
        source, target, move = prepare_icp(pcd_combined, pcd_target)
        
        trans_init = np.asarray([[0, 0, -1, move[0]],
                                [-1, 0, 0, move[1]],
                                [0, 1, 0, move[2]],
                                [0, 0, 0, 1]])
        trans_matrix = do_ICP(source, target, trans_init)
        
        camera_to_mesh_matrix = trans_matrix
        
        mesh_to_camera_matrix = np.linalg.inv(trans_matrix)
        
        return camera_to_mesh_matrix, mesh_to_camera_matrix
    
    def wait_image(self, time_threshold=10.0):
        rospy.logwarn(f"Wait .... (limit: {time_threshold} secs)")
        start_time = time.time()
        
        while time.time() - start_time < time_threshold:
            if self.img_color is not None and self.img_depth is not None:
                rospy.loginfo("Captured!!!")
                return True
            rospy.sleep(1)
            rospy.loginfo("\twait..")
        rospy.logwarn("...Failed...")
        return False
            
        
        
    
    def GetWorldCoordinates(self, request):
        rospy.loginfo(f"Service GetWorldCoordinates {request.target_block}")
        
        resp = GetWorldCoordResponse()
        resp.success = True
        
        if self.tower_transform_initialized is not True:
            resp.success = False
            return resp
        
        is_success = self.wait_image(time_threshold=10.0)
        
        if is_success:
            resp.success = False
            return resp
        
        target_block_color, target_block_label = request.target_block.split()
        blocks_pcd_by_color = self.block_pcd_by_color
        
        if target_block_color == 'init':
            if int(target_block_label) == 1:
                coordinate1 = np.array([37.5, 37.5, 0])
                coordinate2 = np.array([37.5, -37.5, 0])
                push = False
            if int(target_block_label) == 2:
                coordinate1 = np.array([-37.5, 37.5, 0])
                coordinate2 = np.array([-37.5, -37.5, 0])
                push = False
                
        else:
            # CENTER, TARGET, PUSH
            coordinate1, coordinate2, push = get_coordinate(request.target_block, blocks_pcd_by_color, self.camera_to_mesh_matrix)
            
        coordinate1, coordinate2 = transform_coordinates(coordinate1, coordinate2, self.transform_matrix_lst)
        
        
        resp.center_x = coordinate1[0]
        resp.center_y = coordinate1[1]
        resp.center_z = coordinate1[2]
        resp.target_x = coordinate2[0]
        resp.target_y = coordinate2[1]
        resp.target_z = coordinate2[2]
        resp.push = push
            
        return resp
    

if __name__ == '__main__':
    rospy.init_node('service_server_node')  # 노드 초기화
    coordinate_server = CoordinateServer()
    rospy.spin()