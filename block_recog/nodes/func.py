import numpy as np
import cv2
import glob
import open3d as o3d
from matplotlib import pyplot as plt
import copy


def img_masking(img_color, color):
    
    img_hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)

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
    lower_violet = (130-20, 60, 60)
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
        if area < 700:
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
    
    # move the source pcd to do icp
    move = np.array(target_tmp.get_center() - source_tmp.get_center())
        
    return source_tmp, target_tmp, move

def do_ICP(source, target, trans_init):
    
    # evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
    
    reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, 10, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    
    transform_matrix = reg_p2p.transformation
    
    return transform_matrix

def transform_blocks(pcd, icp_transform):
    pcd_temp = copy.deepcopy(pcd)

    pcd_temp.transform(icp_transform)
    
    return pcd_temp


def get_coordinate(target_block, blocks_pcd_by_color, trans):
    target_block_color, target_block_label = target_block.split()
    colors = ['green', 'pink', 'yellow', 'blue', 'violet', 'red']
    
    for col, pcds in zip(colors, blocks_pcd_by_color):
        if col != target_block_color:
            continue
        for idx, pcd in enumerate(pcds):
            if idx != int(target_block_label):
                continue
        
            pcd_new = transform_blocks(pcd, trans)

            
            box_extent = pcd_new.get_axis_aligned_bounding_box().get_extent()
            # print("BOX EXTENT : ", box_extent)
            
            center_coordinate = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points()).mean(axis=0)
            
            # print("BOX CENTER COORDINATE : ", center_coordinate)
            
            # print("BOX MAX X,Y and MEAN Z Coordinate")
            x_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:,0].mean()
            y_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:,1].mean()
            z_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:,2].mean()
            
            if box_extent[1] > 70:
                print("PULL DIRECTION : X")
                push = False

                cen_x = x_mean #- 25/2
                cen_y = y_mean #- 75/2
                cen_z = z_mean
                
                target_x = cen_x + 150
                target_y = cen_y
                target_z = cen_z
                
            elif box_extent[0] > 70:
                print("PULL DIRECTION : Y")
                push = False

                cen_x = x_mean# - 75/2
                cen_y = y_mean# - 25/2
                cen_z = z_mean
                
                target_x = cen_x
                target_y = cen_y + 150
                target_z = cen_z
                
            elif abs(center_coordinate[0]) < 10 and box_extent [1] < 15:
                print("PUSH DIRECTION : Y or -Y")
                push = True

                cen_x = x_mean# - 25/2
                cen_y = y_mean - 75/2
                cen_z = z_mean
                
                target_x = cen_x
                target_y = cen_y + 150
                target_z = cen_z
                
            elif abs(center_coordinate[1]) < 10 and box_extent [0] < 15:
                print("PUSH DIRECTION : X or -X")
                push = True

                cen_x = x_mean - 75/2
                cen_y = y_mean# - 25/2
                cen_z = z_mean
                
                target_x = cen_x + 150
                target_y = cen_y
                target_z = cen_z
                
            elif box_extent[1] < 15:
                print("PULL DIRECTION : -X")
                push = False

                cen_x = x_mean# - 25/2
                cen_y = y_mean - 75/2
                cen_z = z_mean
                
                target_x = cen_x - 150
                target_y = cen_y
                target_z = cen_z
                
            elif box_extent[0] < 15:
                print("PULL DIRECTION : -Y")
                push = False

                cen_x = x_mean - 75/2
                cen_y = y_mean# - 25/2
                cen_z = z_mean
                
                target_x = cen_x
                target_y = cen_y - 150
                target_z = cen_z
                
            else:
                print("NOTHING")
    
    center_coordinate = np.array([cen_x, cen_y, cen_z])            
    target_coordinate = np.array([target_x, target_y, target_z])
    
    return center_coordinate, target_coordinate, push

def coordinate_transform(coordinate, transform_matrix):
    coordinate = copy.deepcopy(coordinate)
    coordinate = np.append(coordinate,1)
    new_coordinate = np.inner(transform_matrix, coordinate)[:3]
    
    return new_coordinate


        
def transform_mat_from_trans_rot(trans, rot):
    e1, e2, e3, e4 = rot
    trans_matrix = np.array([[1-2*(e2**2)-2*(e3**2), 2*(e1*e2-e3*e4), 2*(e1*e3+e2*e4), trans[0]],
                                [2*(e1*e2+e3*e4), 1-2*(e1**2)-2*(e3**2), 2*(e2*e3-e1*e4), trans[1]],
                                [2*(e1*e3-e2*e4), 2*(e2*e3+e1*e4), 1-2*(e1**2)-2*(e2**2), trans[2]],
                                [0, 0, 0, 1]])
    
    return trans_matrix