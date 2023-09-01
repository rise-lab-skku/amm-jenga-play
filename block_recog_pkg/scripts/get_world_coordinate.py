#!/usr/bin/python3

import rospy
from block_recog_pkg.srv import (
    GetWorldCoord,
    GetWorldCoordResponse,
    GetWorldCoordRequest,
    CaptureImage,
    CaptureImageResponse,
    CaptureImageRequest,
)
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

import numpy as np
import time
import func
import pickle
import cv2
import os
from tf_conversions import *



class CoordinateServer:
    """A class that provides services to capture RGB and depth images and obtain target block's world coordinates.

    Attributes:
        tower_transform_initialized (bool): Flag indicating whether the tower's transformation is initialized.
        img_depth (numpy.ndarray): The depth image captured from the camera.
        img_color (numpy.ndarray): The RGB image captured from the camera.
        ready_to_capture_image (bool): Flag indicating whether the server is ready to capture images.
        capture_once (int): Counter to ensure single image capture.

    Subscribers:
        /rgb/image_raw (sensor_msgs.Image): Subscriber for RGB images from the camera.
        /depth_to_rgb/image_raw (sensor_msgs.Image): Subscriber for depth images from the camera.

    Services:
        CaptureImage (block_recog.srv.CaptureImage): Service to capture RGB and depth images.
        GetWorldCoordinates (block_recog.srv.GetWorldCoord): Service to obtain world coordinates for a target block.
    """

    def __init__(self, fake=False):
        self.pkg_path=func.pkg_path
        self.tower_transform_initialized = False

        self.img_depth = None
        self.img_color = None

        self.ready_to_capture_image = False
        self.once_captured = False

        if fake:
            with open(
                os.path.join(self.pkg_path,), "rb"
            ) as f:
                self.img_color = cv2.rotate(pickle.load(f), cv2.ROTATE_180)
            with open(
                os.path.join(self.pkg_path,"data/dep.p"), "rb"
            ) as f:
                self.img_depth = cv2.rotate(pickle.load(f), cv2.ROTATE_180)
        else:
            # Define Subscribers that captures images
            rospy.Subscriber("/rgb/image_raw", Image, self.image_callback_rgb)
            rospy.Subscriber(
                "/depth_to_rgb/image_raw", Image, self.image_callback_depth
            )

        # Define Service that determines whether to capture image or not
        rospy.Service("CaptureImage", CaptureImage, self.capture_flag)

        # Define Service that returns target block's coordinates
        rospy.Service("GetWorldCoordinates", GetWorldCoord, self.GetWorldCoordinates)

    def find_initial_tower_transform(self) -> None:
        """
        Finds the initial transformation matrices for the tower.

        This function performs the following steps:
        1. Masks the input color image to obtain blocks and their masks for each color.
        2. Builds a clean tower point cloud and stores the point clouds of blocks by color.
        3. Calculates the transformation matrix to convert the tower point cloud from mesh frame to camera frame.
        4. Get the transformation matrix to convert the camera frame to the world frame using TF.
        5. Combines the two transformation matrices to get a list of transformation matrices for the tower.

        Note: The function requires self.img_color and self.img_depth to be set with the color and depth images.

        Returns:
            None
        """
        rospy.loginfo("Find Initial Tower Transform")

        blocks_rgb_by_color = []
        blocks_mask_by_color = []

        # Masks the input color image to obtain blocks and their masks for each color.
        for i in range(len(func.colors)):
            blocks_color, blocks_mask = func.mask_image(self.img_color, i)
            blocks_rgb_by_color.append(blocks_color)
            blocks_mask_by_color.append(blocks_mask)

        # Builds a clean tower point cloud and stores the point clouds of blocks by color.
        tower_color = 0

        # Combine masks and color images for each color
        for color in blocks_rgb_by_color:
            for block_c in color:
                tower_color += block_c
        (
            self.pcd_combined,
            self.block_pcd_by_color,
        ) = func.build_clean_tower_pcd_from_blocks(
            blocks_mask_by_color, tower_color, self.img_depth
        )

        # Calculates the transformation matrix to convert the tower point cloud from mesh frame to camera frame.
        trans_matrix=func.calculate_transform_matrix(self.pcd_combined)
        cam2mesh_tf=toTf(fromMatrix(trans_matrix))
        tf.TransformBroadcaster().sendTransform(cam2mesh_tf[0],cam2mesh_tf[1],rospy.Time(0),'mesh',CAM_LINK)

        # Broadcast TF?
        # Broadcat [mesh -> rgb_camera_link]
        # Listen [mesh -> world] TF, not doing transform twice

        # Calculates the transformation matrix to convert the camera frame to the world frame using TF.
        # Wait for lookupTransform to become available
        self.listener = tf.TransformListener()
        rospy.logwarn(
            "Waiting for transform between [panda_link0] and [rgb_camera_link]"
        )
        self.listener.waitForTransform(
            WORLD_LINK, CAM_LINK, rospy.Time(0), rospy.Duration(5)
        )
        self.tower_transform_initialized = True  # Done calculating Transform Matrices


    def GetWorldCoordinates(
        self, request: GetWorldCoordRequest
    ) -> GetWorldCoordResponse:
        """
        Service callback function to get world coordinates of a target block.

        This function calculates the world coordinates of the target block based on its color and label.
        If the tower transformation has not been initialized, the service will return a response with 'success' set to False.
        If target_block_color is 'init', the service will return a response with jenga tower's coordinates and full tower map.

        Args:
            request: The service request object (GetWorldCoordRequest).

        Returns:
            GetWorldCoordResponse: The service response object containing the world coordinates of the target block.
        """
        rospy.loginfo(f"Service GetWorldCoordinates {request.target_block}")

        resp = GetWorldCoordResponse()
        resp.success = True

        # If transform matrix not calculated
        if self.tower_transform_initialized is not True:
            print("false")
            resp.success = False
            return resp

        is_success = self.wait_image(time_threshold=10.0)
        # is_success = True

        if is_success is not True:
            resp.success = False
            return resp

        # target block info from request
        (
            target_block_color,
            target_block_label,
        ) = request.target_block.split()  # e.g. "green 0"
        blocks_pcd_by_color = self.block_pcd_by_color

        # coordinates of jenga tower
        if target_block_color == "init":
            if int(target_block_label) == 1:
                coordinate1 = np.array([0.0375, 0.0375, 0])
                coordinate2 = np.array([0.0375, -0.0375, 0])
            push=False
            if int(target_block_label) == 2:
                coordinate1 = np.array([-0.0375, 0.0375, 0])
                coordinate2 = np.array([-0.0375, -0.0375, 0])
                _, _, _, tower_map = func.get_coordinates(
                    request.target_block,
                    blocks_pcd_by_color,
                    self.cam_to_mesh_transform_matrix,
                )
                push = False
                resp.tower_map = bridge.cv2_to_imgmsg(tower_map.astype("float"))


        else:
            # Returns CENTER, TCP_TARGET coordinates and extract method(push).
            coordinate1, coordinate2, push, tower_map = func.get_coordinates(
                request.target_block,
                blocks_pcd_by_color,
                self.cam_to_mesh_transform_matrix,
            )

        # Failed to calculate coordinates
        if coordinate1 is None:
            resp.success = False
            return resp

        # Transform coordinates to world coordinate system
        resp.center=self.listener.transformPoint('world',PointStamped(header=Header(frame_id='mesh'),point=coordinate1))
        resp.tcp_target=self.listener.transformPoint('world',PointStamped(header=Header(frame_id='mesh'),point=coordinate2))

        # Set method
        if push:
            resp.method = "push"
        else:
            resp.method = "pull"
        # Convert tower map to ROS message

        return resp


if __name__ == "__main__":
    rospy.init_node("coordinate_server_node", anonymous=True)  # init node
    CoordinateServer(fake=True)
    rospy.spin()