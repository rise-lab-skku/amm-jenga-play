#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
import numpy as np
import pickle
import open3d as o3d

from func import *

def handle_transform_matrix(request):
    # 여기에서 실제로 변환 행렬 계산을 수행하고 결과를 반환하는 작업을 수행합니다.
    # 변환 행렬 계산 결과는 response.message에 저장되어 반환됩니다.
    transform_matrix = calculate_transform_matrix()

    response = TriggerResponse()
    if transform_matrix is not None:
        response.success = True
        response.message = str(transform_matrix)
        rospy.loginfo('Transform matrix calculated and sent')
    else:
        response.success = False
        response.message = 'Failed to calculate transform matrix'
        rospy.logwarn('Failed to calculate transform matrix')

    return response

def calculate_transform_matrix():
    
    with open('./dep.p', 'rb') as dep:
        img_depth = pickle.load(dep, encoding="16UC1")
        
    with open('./rgb.p', 'rb') as rgb:
        img_color = pickle.load(rgb)
    
    colors = ['green', 'pink', 'yellow', 'blue', 'violet', 'red']
    blocks_rgb_by_color = []
    blocks_mask_by_color = []
    
    for color in colors:
        blocks_color, blocks_mask = img_masking(img_color, color)
        blocks_rgb_by_color.append(blocks_color)
        blocks_mask_by_color.append(blocks_mask)
        
        
    tower_mask = 0
    tower_color = 0
    for mask, color in zip(blocks_mask_by_color, blocks_rgb_by_color):
        for block_m in mask:
            tower_mask += block_m
        
        for block_c in color:
            tower_color += block_c
            
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.intrinsic_matrix = [[968.813, 0, 1023.83],
                                  [0, 968.635, 775.975],
                                  [0, 0, 1]]
    
    masked_depth = cv2.bitwise_and(img_depth, img_depth, mask = tower_mask)
    tower_pcd = get_pointcloud_from_color_depth(color_image=tower_color, depth_image=masked_depth, intrinsic=intrinsic)
    
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


    mesh_tower = o3d.io.read_triangle_mesh("/mesh/jenga_tower_side_xy.stl")
    mesh_tower.compute_vertex_normals()

    pcd_target = mesh_tower.sample_points_uniformly(number_of_points=len(pcd_combined.points))
    
    source, target, move = prepare_icp(pcd_combined, pcd_target)
    
    trans_init = np.asarray([[0, 0, -1, move[0]],
                             [1, 0, 0, move[1]],
                             [0, -1, 0, move[2]],
                             [0, 0, 0, 1]])
    trans_matrix = do_ICP(source, target, move)
    
    return trans_matrix, blocks_pcd_by_color

if __name__ == '__main__':
    rospy.init_node('processing_node')  # 노드 초기화
    rospy.Service('transform_service', Trigger, handle_transform_matrix)  # 서비스 등록
    rospy.loginfo('Transform service is ready')
    rospy.spin()  # 노드 실행 유지
