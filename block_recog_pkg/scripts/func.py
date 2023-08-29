from typing import Tuple, List, Optional
import numpy as np
import cv2
import open3d as o3d
import copy
import rospy
import os
import yaml
from tf_conversions import *
import rospkg
pkg_path=rospkg.RosPack().get_path("block_recog_pkg")

with open(os.path.join(pkg_path,"data/test.yaml"), "rb") as f:
    colors = yaml.full_load(f)
print(colors)

def img_masking(
    img_color: np.ndarray, color: str
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    Apply color-based image masking on the input color image.

    Parameters:
        img_color (np.ndarray): The input color image in BGR format.
        color (str): The color to be extracted. It should be one of ["red", "pink", "green", "yellow", "blue", "violet"].

    Returns:
        Tuple[List[np.ndarray], List[np.ndarray]]:
        A tuple containing lists of extracted color blocks and their corresponding masks.
            The first list contains the extracted color blocks as numpy arrays.
            The second list contains the corresponding binary masks for each color block.
    """

    

    # Convert to HSV Image
    img_hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)

    val = colors[color]

    img_mask_color = cv2.inRange(img_hsv,np.array(val['lower']),np.array(val['upper']))
    if val['name']=='red' or val['name']=='pink':
        img_mask_color+=cv2.inRange(img_hsv,np.array(val['lower2']),np.array(val['upper2']))
    # The pixels in the range are white, the rest are black

    # Erosion, Dilation for Denoising
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    erosion_image_color = cv2.erode(
        img_mask_color, kernel, iterations=2
    )  # make erosion image
    img_mask_color = cv2.dilate(
        erosion_image_color, kernel, iterations=2
    )  # make dilation image

    # Thresholding
    _, src_bin = cv2.threshold(img_mask_color, 0, 255, cv2.THRESH_OTSU)

    # Seperate Masks using Connnected Pixels
    cnt, labels, stats, _= cv2.connectedComponentsWithStats(src_bin)

    blocks_color = []
    blocks_mask = []

    # Get each block's information. Starting from 1 to exclude the background
    for i in range(1, cnt):
        (x, y, w, h, area) = stats[i]
        block_mask = (labels == i) * img_mask_color
        block_color = cv2.bitwise_and(img_color, img_color, mask=block_mask)

        # For Denoising, skip area under threshold
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

    # Combine masks and color images for each color
    for mask, color in zip(blocks_mask_by_color, blocks_rgb_by_color):
        for block_m in mask:
            tower_mask += block_m

        for block_c in color:
            tower_color += block_c

    return tower_mask, tower_color


def get_pointcloud_from_color_depth(
    color_image: np.ndarray, depth_image: np.ndarray, intrinsic
) -> o3d.geometry.PointCloud:
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

    # Get RGBD Image from RGB Image and Depth Image
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1, depth_trunc=3000.0, convert_rgb_to_intensity=False)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image, depth_image, convert_rgb_to_intensity=False
    )

    # Get PointCloud from RGBD Image
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
    # Combine all point clouds into one
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(all_pcd)):
        pcd_combined += all_pcd[point_id]

    return pcd_combined


def build_clean_tower_pcd_from_blocks(
    blocks_mask_by_color: List[List[np.ndarray]],
    tower_color: np.ndarray,
    img_depth: np.ndarray,
) -> Tuple[o3d.geometry.PointCloud, List[List[o3d.geometry.PointCloud]]]:
    """
    Build a clean tower point cloud from blocks' masks.

    Parameters:
        blocks_mask_by_color (List[List[np.ndarray]]): A list of lists containing masks for each block color.
        tower_color (numpy.ndarray): The color image of the tower.
        img_depth (numpy.ndarray): The depth image of the tower.

    Returns:
        Tuple[o3d.geometry.PointCloud, List[List[o3d.geometry.PointCloud]]]:
        A tuple containing the combined point cloud of the tower
        and a list of lists containing point clouds for each block color.
    """
    # Get Camera Intrinsic Matrix
    # Subscribe 'rgb/camera_info' K ???
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.intrinsic_matrix = [
        [968.813, 0, 1023.83],
        [0, 968.635, 775.975],
        [0, 0, 1],
    ]

    blocks_pcd_by_color = []
    all_pcd = []
    for i, block_mask in enumerate( blocks_mask_by_color  ):
        rospy.loginfo(
            f"Number of Blocks -> Color: {colors[i]['name']}, Recognized Block Numbers: {len(block_mask)}"
        )
        blocks_pcd = []
        for msk in block_mask:
            masked_block_rgb = cv2.bitwise_and(tower_color, tower_color, mask=msk)
            masked_block_depth = cv2.bitwise_and(img_depth, img_depth, mask=msk)

            # Get Each Block's PointCloud
            pcd = get_pointcloud_from_color_depth(
                color_image=masked_block_rgb,
                depth_image=masked_block_depth,
                intrinsic=intrinsic,
            )

            # Remove Outlier Points
            pcd, _ = pcd.remove_radius_outlier(256, 0.025)
            blocks_pcd.append(pcd)
            all_pcd.append(pcd)

        blocks_pcd_by_color.append(blocks_pcd)

    pcd_combined = combine_pcd(all_pcd)

    return pcd_combined, blocks_pcd_by_color


def prepare_icp_move(
    source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud
) -> np.ndarray:
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

    initial_transform = np.asarray(
        [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
    )  # z-axis 180 degree rotation

    # Transform the source PointCloud to align with the target
    source_tmp.transform(initial_transform)

    # Get the translation vector to align the source with the target
    move = np.array(
        target_tmp.get_oriented_bounding_box().get_center()
        - source_tmp.get_oriented_bounding_box().get_center()
    )

    return move


def do_ICP(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    initial_transform: np.ndarray,
) -> np.ndarray:
    """
    Perform Iterative Closest Point (ICP) registration to align the source PointCloud to the target.

    Parameters:
        source (o3d.geometry.PointCloud): The source PointCloud to be aligned to the target.
        target (o3d.geometry.PointCloud): The target PointCloud to which the source will be aligned.
        initial_transform (numpy.ndarray): The initial transformation matrix to seed the ICP algorithm.

    Returns:
        numpy.ndarray: The final transformation matrix after ICP registration.
    """
    print(
        o3d.pipelines.registration.evaluate_registration(
            source, target, 10, initial_transform
        )
    )

    # ICP Registration
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source,
        target,
        10,
        initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000),
    )

    transform_matrix = copy.deepcopy(
        reg_p2p.transformation
    )  # Get the final transformation matrix
    # transform_matrix[0,3] /= 1000
    # transform_matrix[1,3] /= 1000
    # transform_matrix[2,3] /= 1000
    return transform_matrix


def calculate_transform_matrix(
    pcd_combined: o3d.geometry.PointCloud,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute the transformation matrices from the combined point cloud to the camera frame.

    Parameters:
        pcd_combined (open3d.geometry.PointCloud): The combined point cloud.

    Returns:
        Tuple[numpy.ndarray, numpy.ndarray]: A tuple containing two 4x4 transformation matrices.
            The first matrix is the camera-to-mesh transformation matrix.
            The second matrix is the mesh-to-camera transformation matrix.
    """
    # Load jenga tower mesh from stl file
    mesh_tower = o3d.io.read_triangle_mesh(
        os.path.dirname(__file__) + "/../data/jenga_tower_side_xy_m.stl"
    )  # Jenga Tower Mesh
    mesh_tower.compute_vertex_normals()

    # Mesh to Pointcloud
    pcd_tower_from_mesh = copy.deepcopy(
        mesh_tower.sample_points_uniformly(number_of_points=len(pcd_combined.points))
    )
    rospy.loginfo("Start ICP ...")

    # Calculate vector for initial alignment
    move = copy.deepcopy(
        prepare_icp_move(source=pcd_combined, target=pcd_tower_from_mesh)
    )

    # Calculate Transform Matrix by ICP (Camera -> Mesh)
    trans_init = np.asarray(
        [[0, 0, -1, move[0]], [-1, 0, 0, move[1]], [0, 1, 0, move[2]], [0, 0, 0, 1]]
    )
    trans_matrix = do_ICP(
        source=pcd_combined, target=pcd_tower_from_mesh, initial_transform=trans_init
    )

    camera_to_mesh_matrix = trans_matrix  # Transform Matrix (Camera -> Mesh)
    mesh_to_camera_matrix = np.linalg.inv(
        camera_to_mesh_matrix
    )  # Transform Matrix (Mesh -> Camera)

    rospy.loginfo("Done ICP")
    rospy.loginfo(f"camera_to_mesh_matrix: {trans_matrix}")

    return camera_to_mesh_matrix, mesh_to_camera_matrix


def transform_blocks(
    pcd: o3d.geometry.PointCloud, transform_matrix: np.ndarray
) -> o3d.geometry.PointCloud:
    """
    Apply a given transformation matrix to the PointCloud.

    Parameters:
        pcd (o3d.geometry.PointCloud): The input PointCloud to be transformed.
        transform_matrix (numpy.ndarray): The transformation matrix to apply to the PointCloud.

    Returns:
        o3d.geometry.PointCloud: The transformed PointCloud.
    """
    pcd_transformed = copy.deepcopy(pcd)

    # Transform PointCloud
    pcd_transformed.transform(transform_matrix)

    return pcd_transformed


def get_coordinates(
    target_block: str, blocks_pcd_by_color: list, transform_matrix: np.ndarray
) -> Tuple[
    Optional[np.ndarray], Optional[np.ndarray], Optional[bool], Optional[np.ndarray]
]:
    """
    Get the center and TCP (Tool Center Point) target coordinates for a target block.

    Parameters:
        target_block (str): The target block in the format "<color> <label>", e.g., "green 0".
        blocks_pcd_by_color (list): A list of PointClouds grouped by color.
        transform_matrix (numpy.ndarray): The 4x4 transformation matrix used for the target block.

    Returns:
        Tuple[Optional[numpy.ndarray], Optional[numpy.ndarray], Optional[bool], Optional[numpy.ndarray]]:
        A tuple containing the center coordinate, the TCP target coordinate,
        a boolean indicating whether the block should be pushed (True) or pulled (False),
        and an array representing the tower map. If the block does not match any of the conditions, all values will be None.
    """
    # Target Block
    target_block_color, target_block_label = target_block.split()  # e.g. 'green', '0'

    # Define full tower map (12, 3)
    tower_map = [[255 for _ in range(3)] for _ in range(12)]

    # Get Target Block's Coordinates
    for color_idx in range(6):
        col = colors[color_idx]['name']
        pcds = blocks_pcd_by_color[color_idx]
        if target_block_color == "init":
            pass
        elif col != target_block_color:
            continue
        for idx, pcd in enumerate(pcds):
            if target_block_color == "init":
                pass
            elif idx != int(target_block_label):
                continue

            # new_trans[0,3]*=1000
            # new_trans[1,3]*=1000
            # new_trans[2,3]*=1000

            pcd_tower_mesh_coordinate = transform_blocks(pcd, transform_matrix)

            # Get axis aligned bounding box's extents [x, y, z]
            box_extent = (
                pcd_tower_mesh_coordinate.get_axis_aligned_bounding_box().get_extent()
            )

            # Get axis aligned bounding box's center coordinate [x, y, z]
            aabb_center_coordinate = np.array(
                pcd_tower_mesh_coordinate.get_axis_aligned_bounding_box().get_box_points()
            ).mean(axis=0)

            # AABB center coordinate x, y, z
            x_mean = aabb_center_coordinate[0]
            y_mean = aabb_center_coordinate[1]
            z_mean = aabb_center_coordinate[2]

            floors = int(z_mean // 0.015)
            true_z = floors * 0.015 + 0.0075

            # Determine whether to push or pull the block and calculate the target coordinates
            if box_extent[1] > 0.070:
                # print("PULL DIRECTION : X")
                push = False

                cen_x = x_mean
                cen_y = y_mean
                cen_z = true_z

                target_x = cen_x + 0.120
                target_y = cen_y
                target_z = cen_z

                tower_map[floors][0] = color_idx

            elif box_extent[0] > 0.070:
                # print("PULL DIRECTION : Y")
                push = False

                cen_x = x_mean
                cen_y = y_mean
                cen_z = true_z

                target_x = cen_x
                target_y = cen_y + 0.120
                target_z = cen_z

                tower_map[floors][0] = color_idx

            elif abs(aabb_center_coordinate[0]) < 0.010 and box_extent[1] < 0.015:
                # print("PUSH DIRECTION : Y or -Y")
                push = True

                cen_x = x_mean
                cen_y = y_mean - 0.075 / 2
                cen_z = true_z

                target_x = cen_x
                target_y = cen_y + 0.120
                target_z = cen_z

                tower_map[floors][1] = color_idx

            elif abs(aabb_center_coordinate[1]) < 0.010 and box_extent[0] < 0.015:
                # print("PUSH DIRECTION : X or -X")
                push = True

                cen_x = x_mean - 0.075 / 2
                cen_y = y_mean
                cen_z = true_z

                target_x = cen_x + 0.120
                target_y = cen_y
                target_z = cen_z

                tower_map[floors][1] = color_idx

            elif box_extent[1] < 0.015:
                # print("PULL DIRECTION : -X")
                push = False

                cen_x = x_mean
                cen_y = y_mean - 0.075 / 2
                cen_z = true_z

                target_x = cen_x - 0.120
                target_y = cen_y
                target_z = cen_z

                tower_map[floors][2] = color_idx

            elif box_extent[0] < 0.015:
                # print("PULL DIRECTION : -Y")
                push = False

                cen_x = x_mean - 0.075 / 2
                cen_y = y_mean
                cen_z = true_z

                target_x = cen_x
                target_y = cen_y - 0.120
                target_z = cen_z

                tower_map[floors][2] = color_idx

            else:
                print("Cannot Find Box Coordinates")

                return None, None, None, None

    block_center_coordinate = np.array([cen_x, cen_y, cen_z])  # / 1000
    tcp_target_coordinate = np.array([target_x, target_y, target_z])  # / 1000

    return block_center_coordinate, tcp_target_coordinate, push, np.array(tower_map)


def coordinate_transform(
    coordinate3D: np.ndarray, transform_matrix: np.ndarray
) -> np.ndarray:
    """
    Apply a transformation matrix to a 3D coordinate.

    Parameters:
        coordinate (numpy.ndarray): The 3D coordinate as a list or numpy array [x, y, z].
        transform_matrix (numpy.ndarray): The 4x4 transformation matrix to apply.

    Returns:
        numpy.ndarray: The new transformed 3D coordinate as a numpy array [x', y', z'].
    """
    # add 1 to the coordinate to make it homogeneous
    coordinate = np.append(coordinate3D, 1)

    # apply the transformation matrix and remove the homogeneous coordinate
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
    # get rotation quaternion
    e1, e2, e3, e4 = rot

    # create the transformation matrix
    # trans_matrix = np.array(
    #     [
    #         [
    #             1 - 2 * (e2**2) - 2 * (e3**2),
    #             2 * (e1 * e2 - e3 * e4),
    #             2 * (e1 * e3 + e2 * e4),
    #             trans[0],
    #         ],
    #         [
    #             2 * (e1 * e2 + e3 * e4),
    #             1 - 2 * (e1**2) - 2 * (e3**2),
    #             2 * (e2 * e3 - e1 * e4),
    #             trans[1],
    #         ],
    #         [
    #             2 * (e1 * e3 - e2 * e4),
    #             2 * (e2 * e3 + e1 * e4),
    #             1 - 2 * (e1**2) - 2 * (e2**2),
    #             trans[2],
    #         ],
    #         [0, 0, 0, 1],
    #     ]
    # )

    return toMatrix(fromTf((trans,rot)))


def transform_coordinates_to_world(
    coordinate1: np.ndarray,
    coordinate2: np.ndarray,
    transform_matrix_lst: List[np.ndarray],
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Transform two coordinates using a list of transformation matrices.

    Parameters:
        coordinate1 (numpy.ndarray): The first coordinate to be transformed.
        coordinate2 (numpy.ndarray): The second coordinate to be transformed.
        transform_matrix_lst (List[numpy.ndarray]): A list of transformation matrices.

    Returns:
        Tuple[numpy.ndarray, numpy.ndarray]: A tuple containing the transformed coordinate1 and coordinate2.
    """
    rospy.loginfo("transform_coordinates_to_world")

    for trans_mat in transform_matrix_lst:
        coordinate1 = coordinate_transform(coordinate1, trans_mat)

    for trans_mat in transform_matrix_lst:
        coordinate2 = coordinate_transform(coordinate2, trans_mat)

    return coordinate1, coordinate2
