from typing import Tuple, List, Optional, Union
import numpy as np
import cv2
import open3d as o3d
import copy
import rospy


def img_masking(img_color: np.ndarray, color: str) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    Apply color-based image masking on the input color image.

    Parameters:
        img_color (np.ndarray): The input color image in BGR format.
        color (str): The color to be extracted. It should be one of ["red", "pink", "green", "yellow", "blue", "violet"].

    Returns:
        Tuple[List[np.ndarray], List[np.ndarray]]: A tuple containing lists of extracted color blocks and their corresponding masks.
            The first list contains the extracted color blocks as numpy arrays.
            The second list contains the corresponding binary masks for each color block.
    """

    img_hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)

    # RED
    lower_red1 = np.array([0, 100, 50])
    upper_red1 = np.array([15, 255, 255])
    lower_red2 = np.array([160, 100, 50])
    upper_red2 = np.array([179, 255, 255])
    # PINK
    lower_pink1 = np.array([0, 15, 200])
    upper_pink1 = np.array([5, 90, 255])
    lower_pink2 = np.array([150, 15, 200])
    upper_pink2 = np.array([179, 80, 255])
    # GREEN
    lower_green = np.array([70 - 20, 100, 100])
    upper_green = np.array([70 + 15, 255, 255])
    # YELLOW
    lower_yellow = np.array([30 - 10, 60, 120])
    upper_yellow = np.array([30 + 10, 255, 255])
    # BLUE
    lower_blue = np.array([100 - 10, 100, 100])
    upper_blue = np.array([100 + 9, 255, 255])
    # VIOLET
    lower_violet = np.array([130 - 20, 60, 60])
    upper_violet = np.array([130 + 30, 120, 130])

    if color == "pink" or color == "red":
        if color == "pink":
            lower_color1 = lower_pink1
            lower_color2 = lower_pink2
            upper_color1 = upper_pink1
            upper_color2 = upper_pink2
        if color == "red":
            lower_color1 = lower_red1
            lower_color2 = lower_red2
            upper_color1 = upper_red1
            upper_color2 = upper_red2

        mask_color1 = cv2.inRange(img_hsv, lower_color1, upper_color1)
        mask_color2 = cv2.inRange(img_hsv, lower_color2, upper_color2)
        img_mask_color = mask_color1 + mask_color2

    else:
        if color == "blue":
            lower_color = lower_blue
            upper_color = upper_blue
        if color == "green":
            lower_color = lower_green
            upper_color = upper_green
        if color == "violet":
            lower_color = lower_violet
            upper_color = upper_violet
        if color == "yellow":
            lower_color = lower_yellow
            upper_color = upper_yellow

        img_mask_color = cv2.inRange(img_hsv, lower_color, upper_color)  # 범위내의 픽셀들은 흰색, 나머지 검은색

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    erosion_image_color = cv2.erode(img_mask_color, kernel, iterations=2)  # make erosion image
    img_mask_color = cv2.dilate(erosion_image_color, kernel, iterations=2)  # make dilation image

    _, src_bin = cv2.threshold(img_mask_color, 0, 255, cv2.THRESH_OTSU)

    cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(src_bin)

    blocks_color = []
    blocks_mask = []

    for i in range(1, cnt):  # 각각의 객체 정보에 들어가기 위해 반복문. 범위를 1부터 시작한 이유는 배경을 제외
        (x, y, w, h, area) = stats[i]
        block_mask = (labels == i) * img_mask_color
        block_color = cv2.bitwise_and(img_color, img_color, mask=block_mask)

        # 노이즈 제거
        if area < 200:
            continue

        blocks_color.append(block_color)
        blocks_mask.append(block_mask)

    return blocks_color, blocks_mask


def get_tower_mask(
    blocks_mask_by_color: List[np.ndarray], blocks_rgb_by_color: List[np.ndarray]
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate the combined mask and color image of the tower from blocks.

    Parameters:
        blocks_mask_by_color (List[numpy.ndarray]): A list of masks for each color.
        blocks_rgb_by_color (List[numpy.ndarray]): A list of RGB images for each color.

    Returns:
        Tuple[numpy.ndarray, numpy.ndarray]: A tuple containing the combined mask and color image of the tower.
    """
    tower_mask = 0
    tower_color = 0
    for mask, color in zip(blocks_mask_by_color, blocks_rgb_by_color):
        for block_m in mask:
            tower_mask += block_m

        for block_c in color:
            tower_color += block_c

    return tower_mask, tower_color


def get_pointcloud_from_color_depth(color_image: np.ndarray, depth_image: np.ndarray, intrinsic) -> o3d.geometry.PointCloud:
    """
    Generate a 3D point cloud from color and depth images using Open3D library.

    This function takes color and depth images as inputs and generates a 3D point cloud.
    If the input images are in the form of numpy arrays, they will be converted to Open3D
    image format. The color image is expected to be in BGR format.

    Args:
        color_image (numpy array or o3d.geometry.Image): The color image or Open3D image.
        depth_image (numpy array or o3d.geometry.Image): The depth image or Open3D image.
        intrinsic (numpy array): The intrinsic matrix for the camera.

    Returns:
        o3d.geometry.PointCloud: The generated 3D point cloud.
    """
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

    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1, depth_trunc=3000.0, convert_rgb_to_intensity=False)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

    rospy.loginfo("----point cloud is generated----")
    return pcd


def combine_pcd(all_pcd: List[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
    """
    Combine multiple PointCloud objects into a single PointCloud.

    Parameters:
        all_pcd (List[o3d.geometry.PointCloud]): A list of PointCloud objects to be combined.

    Returns:
        o3d.geometry.PointCloud: A new PointCloud object that contains all the points from the input PointClouds.

    Example:
        pcd1 = o3d.geometry.PointCloud()
        pcd2 = o3d.geometry.PointCloud()
        # ... (add points to pcd1 and pcd2)
        combined_pcd = combine_pcd([pcd1, pcd2])
    """
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(all_pcd)):
        pcd_combined += all_pcd[point_id]

    return pcd_combined


def prepare_icp_move(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud) -> np.ndarray:
    """
    Calculate the translation vector for aligning the source PointCloud with the target using ICP.

    Parameters:
        source (o3d.geometry.PointCloud): The source PointCloud to be aligned to the target.
        target (o3d.geometry.PointCloud): The target PointCloud to which the source will be aligned.

    Returns:
        numpy.ndarray: The translation vector (move) to align the source PointCloud with the target.
    """
    source_tmp = copy.deepcopy(source)
    target_tmp = copy.deepcopy(target)

    initial_transform = np.asarray([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

    source_tmp.transform(initial_transform)
    # move the source pcd to do icp
    move = np.array(target_tmp.get_oriented_bounding_box().get_center() - source_tmp.get_oriented_bounding_box().get_center())

    return move


def do_ICP(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud, trans_init: np.ndarray) -> np.ndarray:
    """
    Perform Iterative Closest Point (ICP) registration to align the source PointCloud to the target.

    Parameters:
        source (o3d.geometry.PointCloud): The source PointCloud to be aligned to the target.
        target (o3d.geometry.PointCloud): The target PointCloud to which the source will be aligned.
        trans_init (numpy.ndarray): The initial transformation matrix to seed the ICP algorithm.

    Returns:
        numpy.ndarray: The final transformation matrix after ICP registration.
    """
    print(o3d.pipelines.registration.evaluate_registration(source, target, 10, trans_init))
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source,
        target,
        10,
        trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000),
    )

    transform_matrix = copy.deepcopy(reg_p2p.transformation)
    # transform_matrix[0,3] /= 1000
    # transform_matrix[1,3] /= 1000
    # transform_matrix[2,3] /= 1000
    return transform_matrix


def transform_blocks(pcd: o3d.geometry.PointCloud, icp_transform: np.ndarray) -> o3d.geometry.PointCloud:
    """
    Apply a given transformation matrix to the PointCloud.

    Parameters:
        pcd (o3d.geometry.PointCloud): The input PointCloud to be transformed.
        icp_transform (numpy.ndarray): The transformation matrix to apply to the PointCloud.

    Returns:
        o3d.geometry.PointCloud: The transformed PointCloud.
    """
    pcd_transformed = copy.deepcopy(pcd)

    pcd_transformed.transform(icp_transform)

    return pcd_transformed


def get_coordinate(
    target_block: str, blocks_pcd_by_color: list, transform_matrix: np.ndarray
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[bool]]:
    """
    Get the center and tcp target coordinates for a target block.

    Parameters:
        target_block (str): The target block in the format "<color> <label>", e.g., "green 0".
        blocks_pcd_by_color (list): A list of PointClouds grouped by color.
        trans (numpy.ndarray): The transformation matrix used for the target block.

    Returns:
        Tuple[Optional[numpy.ndarray], Optional[numpy.ndarray], Optional[bool]]: A tuple containing the center coordinate,
        the target coordinate, and a boolean indicating whether the block should be pushed (True) or pulled (False).
        If the block does not match any of the conditions, all values will be None.
    """
    target_block_color, target_block_label = target_block.split()
    colors = ["green", "pink", "yellow", "blue", "violet", "red"]

    for col, pcds in zip(colors, blocks_pcd_by_color):
        if col != target_block_color:
            continue
        for idx, pcd in enumerate(pcds):
            if idx != int(target_block_label):
                continue

            new_trans = copy.deepcopy(transform_matrix)
            # new_trans[0,3]*=1000
            # new_trans[1,3]*=1000
            # new_trans[2,3]*=1000

            pcd_new = transform_blocks(pcd, new_trans)

            box_extent = pcd_new.get_axis_aligned_bounding_box().get_extent()

            center_coordinate = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points()).mean(axis=0)

            # floors = int(center_coordinate[2] // 0.015)

            x_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:, 0].mean()
            y_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:, 1].mean()
            z_mean = np.array(pcd_new.get_axis_aligned_bounding_box().get_box_points())[:, 2].mean()

            if box_extent[1] > 0.070:
                print("PULL DIRECTION : X")
                push = False

                cen_x = x_mean
                cen_y = y_mean
                cen_z = z_mean

                target_x = cen_x + 0.120
                target_y = cen_y
                target_z = cen_z

            elif box_extent[0] > 0.070:
                print("PULL DIRECTION : Y")
                push = False

                cen_x = x_mean
                cen_y = y_mean
                cen_z = z_mean

                target_x = cen_x
                target_y = cen_y + 0.120
                target_z = cen_z

            elif abs(center_coordinate[0]) < 0.010 and box_extent[1] < 0.015:
                print("PUSH DIRECTION : Y or -Y")
                push = True

                cen_x = x_mean
                cen_y = y_mean - 0.075 / 2
                cen_z = z_mean

                target_x = cen_x
                target_y = cen_y + 0.120
                target_z = cen_z

            elif abs(center_coordinate[1]) < 0.010 and box_extent[0] < 0.015:
                print("PUSH DIRECTION : X or -X")
                push = True

                cen_x = x_mean - 0.075 / 2
                cen_y = y_mean
                cen_z = z_mean

                target_x = cen_x + 0.120
                target_y = cen_y
                target_z = cen_z

            elif box_extent[1] < 0.015:
                print("PULL DIRECTION : -X")
                push = False

                cen_x = x_mean
                cen_y = y_mean - 0.075 / 2
                cen_z = z_mean

                target_x = cen_x - 0.120
                target_y = cen_y
                target_z = cen_z

            elif box_extent[0] < 0.015:
                print("PULL DIRECTION : -Y")
                push = False

                cen_x = x_mean - 0.075 / 2
                cen_y = y_mean
                cen_z = z_mean

                target_x = cen_x
                target_y = cen_y - 0.120
                target_z = cen_z

            else:
                print("NOTHING")

                return None, None, None

    block_center_coordinate = np.array([cen_x, cen_y, cen_z])  # / 1000
    tcp_target_coordinate = np.array([target_x, target_y, target_z])  # / 1000

    return block_center_coordinate, tcp_target_coordinate, push


def coordinate_transform(coordinate: Union[list, np.ndarray], transform_matrix: np.ndarray) -> np.ndarray:
    """
    Apply a transformation matrix to a 3D coordinate.

    Parameters:
        coordinate (Union[list, numpy.ndarray]): The 3D coordinate as a list or numpy array [x, y, z].
        transform_matrix (numpy.ndarray): The 4x4 transformation matrix to apply.

    Returns:
        numpy.ndarray: The new transformed 3D coordinate as a numpy array [x', y', z'].
    """
    coordinate = np.append(coordinate, 1)
    new_coordinate = np.inner(transform_matrix, coordinate)[:3]

    return new_coordinate


def transform_mat_from_trans_rot(trans: list, rot: list) -> np.ndarray:
    """
    Create a 4x4 transformation matrix from translation and rotation parameters.

    Parameters:
        trans (list): The translation vector [x, y, z].
        rot (list): The rotation quaternion [e1, e2, e3, e4].

    Returns:
        numpy.ndarray: The 4x4 transformation matrix.
    """
    e1, e2, e3, e4 = rot
    trans_matrix = np.array(
        [
            [
                1 - 2 * (e2**2) - 2 * (e3**2),
                2 * (e1 * e2 - e3 * e4),
                2 * (e1 * e3 + e2 * e4),
                trans[0],
            ],
            [
                2 * (e1 * e2 + e3 * e4),
                1 - 2 * (e1**2) - 2 * (e3**2),
                2 * (e2 * e3 - e1 * e4),
                trans[1],
            ],
            [
                2 * (e1 * e3 - e2 * e4),
                2 * (e2 * e3 + e1 * e4),
                1 - 2 * (e1**2) - 2 * (e2**2),
                trans[2],
            ],
            [0, 0, 0, 1],
        ]
    )
    return trans_matrix
