#!/usr/bin/python3
import rospy
from block_recog_pkg.srv import (
    GetWorldCoord,
    GetWorldCoordResponse,
    # GetWorldCoordRequest,
    CaptureImage,
    CaptureImageResponse,
)
import tf
import numpy as np
import time
import cv2
import open3d as o3d
import func
from typing import Tuple, List

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import copy

bridge = CvBridge()

colors = ["green", "pink", "yellow", "blue", "violet", "red"]


def transform_coordinates_to_world(
    coordinate1: np.ndarray, coordinate2: np.ndarray, transform_matrix_lst: List[np.ndarray]
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
        coordinate1 = func.coordinate_transform(coordinate1, trans_mat)

    for trans_mat in transform_matrix_lst:
        coordinate2 = func.coordinate_transform(coordinate2, trans_mat)

    return coordinate1, coordinate2


class CoordinateServer:
    """A class that provides services to capture RGB and depth images and obtain target block's world coordinates.

    Attributes:
        mesh_tower (open3d.geometry.TriangleMesh): The mesh of the jenga tower.
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

    def __init__(self):
        self.mesh_tower = o3d.io.read_triangle_mesh("../block_recog/mesh/jenga_tower_side_xy_m.stl")  # Jenga Tower Mesh
        self.mesh_tower.compute_vertex_normals()

        self.tower_transform_initialized = False

        self.img_depth = None
        self.img_color = None

        self.ready_to_capture_image = False
        self.capture_once = 0

        rospy.logwarn("Define subscribers")
        rospy.Subscriber("/rgb/image_raw", Image, self.image_callback_rgb)
        rospy.Subscriber("/depth_to_rgb/image_raw", Image, self.image_callback_depth)

        # rospy.logwarn("Define capture service")
        capture_service = rospy.Service("CaptureImage", CaptureImage, self.capture_flag)

        # rospy.logwarn("Define get world coordinate service")
        block_coordinate_service = rospy.Service("GetWorldCoordinates", GetWorldCoord, self.GetWorldCoordinates)

    def find_initial_tower_transform(self):
        """
        Finds the initial transformation matrices for the tower.

        This function performs the following steps:
        1. Masks the input color image to obtain blocks and their masks for each color.
        2. Builds a clean tower point cloud and stores the point clouds of blocks by color.
        3. Calculates the transformation matrix to convert the tower point cloud from mesh frame to camera frame.
        4. Calculates the transformation matrix to convert the camera frame to the world frame using TF.
        5. Combines the two transformation matrices to get a list of transformation matrices for the tower.

        Note: The function requires self.img_color and self.img_depth to be set with the color and depth images.

        Returns:
            None
        """
        rospy.loginfo("find_initial_tower_transform")

        colors = ["green", "pink", "yellow", "blue", "violet", "red"]
        blocks_rgb_by_color = []
        blocks_mask_by_color = []

        for color in colors:
            blocks_color, blocks_mask = func.img_masking(self.img_color, color)
            blocks_rgb_by_color.append(blocks_color)
            blocks_mask_by_color.append(blocks_mask)

        tower_mask, tower_color = func.get_tower_mask(blocks_mask_by_color, blocks_rgb_by_color)

        self.pcd_combined, self.block_pcd_by_color = self.build_clean_tower_pcd_from_blocks(
            blocks_mask_by_color, tower_color, self.img_depth
        )

        self.mesh_to_cam_transform_matrix = self.transform_matrix_mesh_to_camera(self.pcd_combined)
        # Broadcast TF?
        # Broadcat [mesh -> rgb_camera_link]
        # Listen [mesh -> world] TF, not doing transform twice

        # Wait for lookupTransform to become available
        listener = tf.TransformListener()
        rospy.logwarn("Waiting for transform between [panda_link0] and [rgb_camera_link]")
        listener.waitForTransform("panda_link0", "rgb_camera_link", rospy.Time(0), rospy.Duration(5.0))

        try:
            (trans_cam_to_world, rot_cam_to_world) = listener.lookupTransform("panda_link0", "rgb_camera_link", rospy.Time(0))
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            raise

        self.cam_to_world_transform_matrix = func.transform_mat_from_trans_rot(trans_cam_to_world, rot_cam_to_world)

        rospy.loginfo(f"cam_to_world_transform_matrix : {self.cam_to_world_transform_matrix}")

        self.transform_matrix_lst = [
            self.mesh_to_cam_transform_matrix,
            self.cam_to_world_transform_matrix,
        ]

        self.tower_transform_initialized = True  # Done calculating Transform Matrices

    def capture_flag(self, request):
        """
        Captures an image and initializes the tower transformation.

        This function performs the following steps:
        1. Checks if the image has already been captured; if yes, it returns 'SKIPPED'.
        2. Sets the flag 'ready_to_capture_image' to True, indicating that an image capture is requested.
        3. Waits for the image capture to complete with a time threshold of 10 seconds (you may need to adjust this value).
        4. Calls 'find_initial_tower_transform()' to calculate the initial transformation matrices for the tower.
        5. If the image capture and transformation calculation are successful, it returns 'SUCCESS'.
        Otherwise, it returns 'FAILED'.

        Args:
            request: The service request object (CaptureImageRequest).

        Returns:
            CaptureImageResponse: The service response object containing the capture status (SUCCESS, FAILED, or SKIPPED).
        """
        rospy.loginfo("Service CaptureImage")
        print("capture_flag")

        resp = CaptureImageResponse()

        if self.capture_once > 0:
            resp.status = resp.SKIPPED
            return resp

        self.ready_to_capture_image = True
        is_success = self.wait_image(time_threshold=10)
        # is_success = True

        if is_success:
            self.find_initial_tower_transform()
            resp.status = resp.SUCCESS
            self.capture_once += 1
            return resp

        resp.status = resp.FAILED
        return resp

    def image_callback_rgb(self, msg) -> None:
        """
        Callback function to receive and store RGB images.

        Parameters:
            msg (sensor_msgs.msg.Image): The ROS message containing the RGB image.

        Returns:
            None
        """
        if self.img_color is None and self.ready_to_capture_image:
            self.img_color = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rospy.logwarn(f"RGB image captured (self.img_color.shape: {self.img_color.shape}))")

    def image_callback_depth(self, msg) -> None:
        """
        Callback function to receive and store depth images.

        Parameters:
            msg (sensor_msgs.msg.Image): The ROS message containing the depth image.

        Returns:
            None
        """
        if self.img_depth is None and self.ready_to_capture_image:
            self.img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            rospy.logwarn(f"Depth image captured (self.img_depth.shape: {self.img_depth.shape}))")

    def build_clean_tower_pcd_from_blocks(
        self, blocks_mask_by_color: List[List[np.ndarray]], tower_color: np.ndarray, img_depth: np.ndarray
    ) -> Tuple[o3d.geometry.PointCloud, List[List[o3d.geometry.PointCloud]]]:
        """
        Build a clean tower point cloud from blocks' masks.

        Parameters:
            blocks_mask_by_color (List[List[np.ndarray]]): A list of lists containing masks for each block color.
            tower_color (numpy.ndarray): The color image of the tower.
            img_depth (numpy.ndarray): The depth image of the tower.

        Returns:
            Tuple[o3d.geometry.PointCloud, List[List[o3d.geometry.PointCloud]]]: A tuple containing the combined point cloud of the tower and a list of lists containing point clouds for each block color.
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
        for color, block_mask in zip(colors, blocks_mask_by_color):
            rospy.loginfo(f"Number of Blocks -> Color: {color}, Recognized Block Numbers: {len(block_mask)}")
            blocks_pcd = []
            for msk in block_mask:
                masked_block_rgb = cv2.bitwise_and(tower_color, tower_color, mask=msk)
                masked_block_depth = cv2.bitwise_and(img_depth, img_depth, mask=msk)

                # Get Each Block's PointCloud
                pcd = func.get_pointcloud_from_color_depth(
                    color_image=masked_block_rgb,
                    depth_image=masked_block_depth,
                    intrinsic=intrinsic,
                )

                # Remove Outlier Points
                pcd, _ = pcd.remove_radius_outlier(256, 0.025)
                blocks_pcd.append(pcd)
                all_pcd.append(pcd)

            blocks_pcd_by_color.append(blocks_pcd)

        pcd_combined = func.combine_pcd(all_pcd)

        return pcd_combined, blocks_pcd_by_color

    def transform_matrix_mesh_to_camera(self, pcd_combined: o3d.geometry.PointCloud) -> np.ndarray:
        """
        Compute the transformation matrix from the combined point cloud to the camera frame.

        Parameters:
            pcd_combined (open3d.geometry.PointCloud): The combined point cloud.

        Returns:
            numpy.ndarray: The 4x4 transformation matrix from the combined point cloud to the camera frame.
        """
        pcd_c = copy.deepcopy(pcd_combined)
        pcd_target = copy.deepcopy(self.mesh_tower.sample_points_uniformly(number_of_points=len(pcd_c.points)))
        rospy.loginfo("Start ICP ...")

        source = pcd_c
        target = pcd_target
        move = copy.deepcopy(func.prepare_icp_move(source=source, target=target))

        trans_init = np.asarray([[0, 0, -1, move[0]], [-1, 0, 0, move[1]], [0, 1, 0, move[2]], [0, 0, 0, 1]])
        trans_matrix = func.do_ICP(source, target, trans_init)  # Transform Matrix (Camera -> Mesh)

        self.camera_to_mesh_matrix = trans_matrix

        rospy.loginfo("Done ICP")
        rospy.loginfo(f"camera_to_mesh_matrix: {trans_matrix}")

        return np.linalg.inv(trans_matrix)  # Transform Matrix (Mesh -> Camera)

    def wait_image(self, time_threshold: float = 10.0) -> bool:
        """
        Wait for the RGB and depth images to be captured.

        This function waits for the RGB and depth images to be available for a specified time threshold.
        If both images are captured within the time threshold, the function returns True.
        If the time threshold is reached and the images are not captured, the function returns False.

        Args:
            time_threshold (float): The maximum time to wait for the images to be captured, in seconds. Default is 10.0.

        Returns:
            bool: True if both RGB and depth images are captured within the time threshold, False otherwise.
        """
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
        """
        Service function to get world coordinates of a target block.

        This function calculates the world coordinates of the target block based on its color and label.
        If the tower transformation has not been initialized, the service will return a response with 'success' set to False.

        Args:
            request: The service request object (GetWorldCoordRequest).

        Returns:
            GetWorldCoordResponse: The service response object containing the world coordinates of the target block.
        """
        rospy.loginfo(f"Service GetWorldCoordinates {request.target_block}")

        resp = GetWorldCoordResponse()
        resp.success = True

        if self.tower_transform_initialized is not True:
            print("false")
            resp.success = False
            return resp

        is_success = self.wait_image(time_threshold=10.0)
        # is_success = True

        if is_success is not True:
            resp.success = False
            return resp

        target_block_color, target_block_label = request.target_block.split()
        blocks_pcd_by_color = self.block_pcd_by_color

        # coordinates of jenga tower
        if target_block_color == "init":
            if int(target_block_label) == 1:
                coordinate1 = np.array([0.0375, 0.0375, 0])
                coordinate2 = np.array([0.0375, -0.0375, 0])
                push = False
            if int(target_block_label) == 2:
                coordinate1 = np.array([-0.0375, 0.0375, 0])
                coordinate2 = np.array([-0.0375, -0.0375, 0])
                push = False

        else:
            # CENTER, TARGET, PUSH
            coordinate1, coordinate2, push = func.get_coordinate(
                request.target_block, blocks_pcd_by_color, self.camera_to_mesh_matrix
            )

        coordinate1, coordinate2 = transform_coordinates_to_world(coordinate1, coordinate2, self.transform_matrix_lst)

        resp.center_x = coordinate1[0]
        resp.center_y = coordinate1[1]
        resp.center_z = coordinate1[2]
        resp.target_x = coordinate2[0]
        resp.target_y = coordinate2[1]
        resp.target_z = coordinate2[2]
        resp.push = push

        return resp


if __name__ == "__main__":
    rospy.init_node("service_server_node")  # init node
    coordinate_server = CoordinateServer()
    rospy.spin()
