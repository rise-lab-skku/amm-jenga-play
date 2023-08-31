from typing import Tuple, List, Optional
import numpy as np
import cv2
import open3d as o3d
import copy
import rospy
import os
import yaml
import rospkg

pkg_path = rospkg.RosPack().get_path("block_recog_pkg")
colors = yaml.full_load(open(os.path.join(pkg_path, "data/colors.yaml"), "rb"))
BLOCK_X=0.075
BLOCK_Y=0.025
BLOCK_Z=0.015
def mask_image(
    img_rgb: np.ndarray, id: int
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

    color = colors[id]
    img_mask_color = cv2.inRange(
        img_hsv, np.array(color["lower"]), np.array(color["upper"])
    )

    # Erosion, Dilation for Denoising
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    erosion_image_color = cv2.erode(
        img_mask_color, kernel, iterations=2
    )  # make erosion image
    img_mask_color = cv2.dilate(
        erosion_image_color, kernel, iterations=2
    )  # make dilation image

    cnt,labels= cv2.connectedComponents(img_mask_color)

    blocks_color = []
    blocks_mask = []

    for i in range(1, cnt):
        block_mask = (labels == i) * img_mask_color
        block_color = cv2.bitwise_and(img_rgb, img_rgb, mask=block_mask)
        blocks_color.append(block_color)
        blocks_mask.append(block_mask)

    return blocks_color, blocks_mask

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
    for i, block_mask in enumerate(blocks_mask_by_color):
        rospy.loginfo(
            f"Number of Blocks -> Color: {colors[i]['name']}, Recognized Block Numbers: {len(block_mask)}"
        )
        blocks_pcd = []
        for msk in block_mask:
            rgb_image=o3d.geometry.Image(cv2.bitwise_and(tower_color, tower_color, mask=msk))
            d_image=o3d.geometry.Image(cv2.bitwise_and(img_depth, img_depth, mask=msk))
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_image, d_image, convert_rgb_to_intensity=False)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

            rospy.loginfo("----point cloud is generated----")

            # Remove Outlier Points
            pcd, _ = pcd.remove_radius_outlier(256, 0.025)
            blocks_pcd.append(pcd)
            all_pcd.append(pcd)

        blocks_pcd_by_color.append(blocks_pcd)

    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(all_pcd)):
        pcd_combined += all_pcd[point_id]

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

    # ICP Registration
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source,
        target,
        10,
        initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000),
    )

    
    return reg_p2p.transformation


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
       os.path.join(pkg_path,"data/jenga_tower_side_xy_m.stl")
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


    rospy.loginfo("Done ICP")
    rospy.loginfo(f"camera_to_mesh_matrix: {trans_matrix}")

    return trans_matrix

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
    for color_idx in range(len):
        col = colors[color_idx]["name"]
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

    block_center_coordinate = Point([cen_x, cen_y, cen_z])  # / 1000
    tcp_target_coordinate = Point([target_x, target_y, target_z])  # / 1000

    return block_center_coordinate, tcp_target_coordinate, push, np.array(tower_map)
