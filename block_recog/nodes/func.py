import numpy as np
import cv2
import glob
import open3d as o3d
from matplotlib import pyplot as plt
import copy


initial_transform = np.asarray([[0, 0, -1, 0],
                                    [1, 0, 0, 0],
                                    [0, -1, 0, 0],
                                    [0, 0, 0, 1]])

def img_masking(img_hsv, img_color, color):
    
    # RED
    lower_red1 = np.array([0, 130, 50])
    upper_red1 = np.array([15, 255, 255])
    lower_red2 = np.array([160,130,50])
    upper_red2 = np.array([179,255,255])

    # PINK
    lower_pink1 = np.array([0, 70, 80])
    upper_pink1 = np.array([10, 130, 255])
    lower_pink2 = np.array([150,70,80])
    upper_pink2 = np.array([179,130,255])

    # GREEN
    lower_green = (70-20, 50, 50)
    upper_green = (70+15, 255, 255)

    # YELLOW
    lower_yellow = (30-10, 80, 80)
    upper_yellow = (30+10, 255, 255)

    # BLUE
    lower_blue = (100-10, 100, 100)
    upper_blue = (100+9, 255, 255)

    # VIOLET
    lower_violet = (130-20, 50, 30)
    upper_violet = (130+20, 255, 255)
    
    if color == 'pink' or color =='red':
        if color == 'pink':
            lower_color1 = lower_pink1
            lower_color2 = lower_pink2
            upper_color1 = upper_pink1
            upper_color2 = upper_pink2
        if color == 'red':
            lower_color1 = lower_red1
            lower_color2 = lower_red2
            upper_color1 = upper_red1
            upper_color2 = upper_red2

        mask_color1 = cv2.inRange(img_hsv, lower_color1, upper_color1)
        mask_color2 = cv2.inRange(img_hsv, lower_color2, upper_color2)
        img_mask_color = mask_color1 + mask_color2

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        erosion_image_color = cv2.erode(img_mask_color, kernel, iterations=2)  #// make erosion image
        img_mask_color = cv2.dilate(erosion_image_color, kernel, iterations=2)  #// make dilation image

        # 바이너리 이미지를 마스크로 사용하여 원본이미지에서 범위값에 해당하는 영상부분을 획득
        img_result_color = cv2.bitwise_and(img_color, img_color, mask = img_mask_color) 
        
        # exec(f"img_result_{color} = img_result_color")
    
    else:
        if color == 'blue':
            lower_color = lower_blue
            upper_color = upper_blue
        if color == 'green':
            lower_color = lower_green
            upper_color = upper_green
        if color == 'violet':
            lower_color = lower_violet
            upper_color = upper_violet
        if color == 'yellow':
            lower_color = lower_yellow
            upper_color = upper_yellow

        img_mask_color = cv2.inRange(img_hsv, lower_color, upper_color) # 범위내의 픽셀들은 흰색, 나머지 검은색

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        erosion_image_color = cv2.erode(img_mask_color, kernel, iterations=2)  #// make erosion image
        img_mask_color = cv2.dilate(erosion_image_color, kernel, iterations=2)  #// make dilation image

        # 바이너리 이미지를 마스크로 사용하여 원본이미지에서 범위값에 해당하는 영상부분을 획득
        img_result_color = cv2.bitwise_and(img_color, img_color, mask = img_mask_color) 

        # exec(f"img_result_{color} = img_result_color")
        
    _, src_bin = cv2.threshold(img_mask_color, 0, 255, cv2.THRESH_OTSU)
    each_color_filtered = cv2.bitwise_and(img_color, img_color, mask = src_bin)

    cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(src_bin)

    blocks_color = []
    blocks_mask = []

    for i in range(1, cnt): # 각각의 객체 정보에 들어가기 위해 반복문. 범위를 1부터 시작한 이유는 배경을 제외
        (x, y, w, h, area) = stats[i]
        cen_x, cen_y = map(int, centroids[i])
        block_mask = (labels==i)*img_mask_color
        block_color = cv2.bitwise_and(img_color, img_color, mask = block_mask)
        
        # 노이즈 제거
        if area < 600:
            continue
        
        
        blocks_color.append(block_color)
        blocks_mask.append(block_mask)
    
    return blocks_color, blocks_mask
        

def get_pointcloud_from_color_depth(color_image, depth_image, intrinsic):
    o3d_img = o3d.geometry.Image()
    
    if isinstance(color_image, type(o3d_img)):
        pass
    elif isinstance(color_image, np.ndarray):
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        color_image = o3d.geometry.Image(color_image)
        
    if isinstance(depth_image, type(o3d_img)):
        pass
    elif isinstance(depth_image, np.ndarray):
        depth_image = o3d.geometry.Image(depth_image)
        
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    
    return pcd

def combine_pcd(all_pcd):
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(all_pcd)):
        pcd_combined += all_pcd[point_id]
        
    return pcd_combined

def prepare_icp(source, target):
    
    source_tmp = copy.deepcopy(source)
    target_tmp = copy.deepcopy(target)
    
    # make the point cloud into right position
    source_tmp.transform(initial_transform)
    
    # resize the target pointcloud to make two pointclouds into same scale
    resize = (np.array(target_tmp.points)[:,2].max() - np.array(target_tmp.points)[:,2].min()) / (np.array(source_tmp.points)[:,2].max() - np.array(source_tmp.points)[:,2].min())
    
    # move the source pcd to do icp
    move = np.array(target_tmp.get_center() - source_tmp.get_center()*resize)
    
    # a = o3d.cpu.pybind.utility.Vector3dVector(np.array(target_tmp.points))
    b = o3d.cpu.pybind.utility.Vector3dVector(np.array(source_tmp.points)*resize + move)
    
    # target_tmp.points = a
    source_tmp.points = b
    
    return source_tmp, target_tmp, resize, move

def do_ICP(source, target, threshold):
    trans_init = np.asarray([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
    
    # evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
    
    reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20000))
    
    transform_matrix = reg_p2p.transformation
    
    return transform_matrix

def transform_blocks(pcd, icp_transform, resize, move):
    pcd_temp = copy.deepcopy(pcd)
    pcd_temp.transform(initial_transform)
    aa = o3d.cpu.pybind.utility.Vector3dVector(np.array(pcd_temp.points)*resize + move)
    pcd_temp.points = aa
    pcd_temp.transform(icp_transform)
    
    return pcd_temp
