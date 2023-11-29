from typing import Tuple, List, Optional
import numpy as np
import cv2
import open3d as o3d
import copy
import rospy
import os
from utils import *


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
