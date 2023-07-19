#!/usr/bin/python3
import rospy
from block_recog_pkg.srv import (
    GetWorldCoord,
    GetWorldCoordResponse,
    GetWorldCoordRequest,
    CaptureImage,
    CaptureImageResponse,
)
import tf
import numpy as np
import time
import cv2
import open3d as o3d
import func

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pickle
import copy

bridge = CvBridge()

colors = colors = ["green", "pink", "yellow", "blue", "violet", "red"]


def transform_coordinates(coordinate1, coordinate2, transform_matrix_lst):

    rospy.loginfo("transform_coordinates")
    rospy.loginfo(f"coordinate1 : {coordinate1}")
    for trans_mat in transform_matrix_lst:
        coordinate1 = func.coordinate_transform(coordinate1, trans_mat)

        rospy.loginfo(f"coordinate1 : {coordinate1}")

    for trans_mat in transform_matrix_lst:
        coordinate2 = func.coordinate_transform(coordinate2, trans_mat)

    return coordinate1, coordinate2


class CoordinateServer:
    def __init__(self):
        self.mesh_tower = o3d.io.read_triangle_mesh(
            "/home/hr/Desktop/urp2/amm-jenga-play/block_recog/mesh/jenga_tower_side_xy.stl")
        self.mesh_tower.compute_vertex_normals()

        # jenga_first_coord1 = np.array([0.0375, 0.0375, 0])
        # jenga_first_coord2 = np.array([0.0375, -0.0375, 0])
        # jenga_first_coord3 = np.array([-0.0375, -0.0375, 0])
        # jenga_first_coord4 = np.array([-0.0375, 0.0375, 0])

        # self.jenga_first_coord = [jenga_first_coord1, jenga_first_coord2, jenga_first_coord3, jenga_first_coord4]

        self.tower_transform_initialized = False
        # -------------------------------------------------------------------------

        # with open('/home/shs/catkin_ws/src/block_recog/img/dep.p', 'rb') as dep:
        #     img_depth = pickle.load(dep, encoding="16UC1")

        # with open('/home/shs/catkin_ws/src/block_recog/img/rgb.p', 'rb') as rgb:
        #     img_color = pickle.load(rgb)
        self.img_depth = None
        self.img_color = None


        # with open('/home/hr/Desktop/rgb.p', 'rb') as rgb:
        #     self.img_color = pickle.load(rgb, encoding="bgr8")
        # with open('/home/hr/Desktop/dep.p', 'rb') as dep:
        #     self.img_depth = pickle.load(dep, encoding="16UC1")

        self.ready_to_capture_image = False
        self.capture_once = 0

        rospy.logwarn("Define subscribers")
        rospy.Subscriber("/rgb/image_raw", Image, self.image_callback1)
        rospy.Subscriber("/depth_to_rgb/image_raw", Image, self.image_callback2)

        # rospy.logwarn("Define capture service")
        capture_service = rospy.Service("CaptureImage", CaptureImage, self.capture_flag)

        # rospy.logwarn("Define get world coordinate service")
        service = rospy.Service("GetWorldCoordinates", GetWorldCoord, self.GetWorldCoordinates)

    def find_initial_tower_transform(self):
        rospy.loginfo("find_initial_tower_transform")

        colors = ["green", "pink", "yellow", "blue", "violet", "red"]
        blocks_rgb_by_color = []
        blocks_mask_by_color = []

        for color in colors:
            blocks_color, blocks_mask = func.img_masking(self.img_color, color)
            blocks_rgb_by_color.append(blocks_color)
            blocks_mask_by_color.append(blocks_mask)

        # --------------------------------------------------------------------------

        tower_mask, tower_color = self.get_tower_mask(blocks_mask_by_color, blocks_rgb_by_color)

        rospy.loginfo("self.get_tower_mask")
        # masked_depth = cv2.bitwise_and(img_depth, img_depth, mask = tower_mask)
        # tower_pcd = get_pointcloud_from_color_depth(color_image=tower_color, depth_image=masked_depth, intrinsic=intrinsic)

        # --------------------------------------------------------------------------
        self.pcd_combined, self.block_pcd_by_color = self.build_clean_tower_pcd_from_blocks(
            blocks_mask_by_color, tower_color, self.img_depth
        )
        # o3d.visualization.draw_geometries([self.pcd_combined])

        rospy.loginfo("self.block_pcd_by_color")
        # --------------------------------------------------------------------------

        self.trans_matrix = self.transform_matrix_mesh_to_camera(self.pcd_combined)

            # rospy.logwarn(f"Dept
        # --------------------------------------------------------------------------

        # Wait for lookupTransform to become available
        listener = tf.TransformListener()
        rospy.logwarn("Waiting for transform between [panda_link0] and [rgb_camera_link]")
        listener.waitForTransform("panda_link0", "rgb_camera_link", rospy.Time(0), rospy.Duration(5.0))
        # listener.waitForTransform("rgb_camera_link", "panda_link0", rospy.Time(0), rospy.Duration(5.0))
        try:
            # lookupTransform('target', 'source', Time)
            (trans_cam_to_world, rot_cam_to_world) = listener.lookupTransform("panda_link0", "rgb_camera_link", rospy.Time(0))
            # (trans_cam_to_world, rot_cam_to_world) = listener.lookupTransform("rgb_camera_link", "panda_link0",  rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            raise

        self.cam_to_world_transform_matrix = func.transform_mat_from_trans_rot(trans_cam_to_world, rot_cam_to_world)

        rospy.loginfo(f"self.cam_to_world_transform_matrix : {self.cam_to_world_transform_matrix}")

        self.transform_matrix_lst = [self.trans_matrix, self.cam_to_world_transform_matrix]

        self.tower_transform_initialized = True

    def capture_flag(self, request):
        rospy.loginfo("Service CaptureImage")
        print('capture_flag')

        resp = CaptureImageResponse()

        if self.capture_once > 0:
            resp.status = resp.SKIPPED
            return resp

        self.ready_to_capture_image = True
        is_success = self.wait_image(time_threshold=10)
        is_success = True

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
            self.img_color = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rospy.logwarn(f"RGB image captured (self.img_color.shape: {self.img_color.shape}))")
        # with open('/home/hr/Desktop/rgb.p', 'rb') as rgb:
        #     self.img_color = pickle.load(rgb, encoding="bgr8")
        # Save pickle
        # with open('/home/shs/catkin_ws/src/block_recog/img/rgb.p', 'wb') as rgb:
        #     pickle.dump(img_color, rgb)

    def image_callback2(self, msg):
        # Convert ROS images to OpenCV format
        if self.img_depth is None and self.ready_to_capture_image:
            self.img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            rospy.logwarn(f"Depth image captured (self.img_depth.shape: {self.img_depth.shape}))")
        # with open('/home/hr/Desktop/dep.p', 'rb') as dep:
        #     self.image_depth = pickle.load(dep, encoding="16UC1")
        # Save pickle
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
        intrinsic.intrinsic_matrix = [[968.813, 0, 1023.83], [0, 968.635, 775.975], [0, 0, 1]]

        blocks_pcd_by_color = []
        all_pcd = []
        for color, block_mask in zip(colors, blocks_mask_by_color):
            rospy.loginfo(f"build_clean_tower_pcd_from_blocks. color: {color}, len(block_mask): {len(block_mask)}")
            blocks_pcd = []
            for msk in block_mask:
                masked_block_rgb = cv2.bitwise_and(tower_color, tower_color, mask=msk)
                masked_block_depth = cv2.bitwise_and(img_depth, img_depth, mask=msk)

                # Get Each Block's PointCloud
                pcd = func.get_pointcloud_from_color_depth(
                    color_image=masked_block_rgb, depth_image=masked_block_depth, intrinsic=intrinsic
                )

                # Remove Outlier Points
                pcd, _ = pcd.remove_radius_outlier(256, 25)
                blocks_pcd.append(pcd)
                all_pcd.append(pcd)

                rospy.loginfo(f"pcd info : {pcd}")

            blocks_pcd_by_color.append(blocks_pcd)

        pcd_combined = func.combine_pcd(all_pcd)

        return pcd_combined, blocks_pcd_by_color

    def transform_matrix_mesh_to_camera(self, pcd_combined):
        # mesh_tower = o3d.io.read_triangle_mesh("/home/hr/Desktop/urp2/amm-jenga-play/block_recog/mesh/jenga_tower_side_xy.stl")
        # mesh_tower.compute_vertex_normals()

        pcd_c = copy.deepcopy(pcd_combined)
        pcd_target = copy.deepcopy(self.mesh_tower.sample_points_uniformly(number_of_points=len(pcd_c.points)))
        rospy.loginfo(f"mesh info : [{pcd_target}]")
        rospy.loginfo("Start ICP")

        source, target, move = copy.deepcopy(func.prepare_icp(pcd_c, pcd_target))
        print(move)

        trans_init = np.asarray([[0, 0, -1, move[0]], [-1, 0, 0, move[1]], [0, 1, 0, move[2]], [0, 0, 0, 1]])
        trans_matrix = func.do_ICP(source, target, trans_init)

        self.camera_to_mesh_matrix = trans_matrix

        rospy.loginfo(f"camera_to_mesh_matrix: {trans_matrix}")

        # mesh_to_camera_matrix = np.linalg.inv(trans_matrix)
        # func.draw_registration_result(source, target, trans_matrix)

        return np.linalg.inv(trans_matrix)   #camera_to_mesh_matrix, mesh_to_camera_matrix

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
            print('false')
            resp.success = False
            return resp

        # is_success = self.wait_image(time_threshold=10.0)
        is_success = True

        if is_success is not True:
            resp.success = False
            return resp

        target_block_color, target_block_label = request.target_block.split()
        blocks_pcd_by_color = self.block_pcd_by_color

        if target_block_color == "init":
            if int(target_block_label) == 1:
                print('true')
                coordinate1 = np.array([0.0375, 0.0375, 0])
                coordinate2 = np.array([0.0375, -0.0375, 0])
                push = False
            if int(target_block_label) == 2:
                coordinate1 = np.array([-0.0375, 0.0375, 0])
                coordinate2 = np.array([-0.0375, -0.0375, 0])
                push = False

        else:
            print(blocks_pcd_by_color)
            # CENTER, TARGET, PUSH
            coordinate1, coordinate2, push = func.get_coordinate(
                request.target_block, blocks_pcd_by_color, self.camera_to_mesh_matrix
            )
        print('hello world')
        coordinate1, coordinate2 = transform_coordinates(coordinate1, coordinate2, self.transform_matrix_lst)

        resp.center_x = coordinate1[0]
        resp.center_y = coordinate1[1]
        resp.center_z = coordinate1[2]
        resp.target_x = coordinate2[0]
        resp.target_y = coordinate2[1]
        resp.target_z = coordinate2[2]
        resp.push = push

        return resp


if __name__ == "__main__":
    rospy.init_node("service_server_node")  # 노드 초기화
    coordinate_server = CoordinateServer()
    # request = GetWorldCoordRequest()
    # request.target_block = "init 1"
    # coordinate_server.find_initial_tower_transform()
    # coordinate_server.GetWorldCoordinates(request)
    rospy.spin()
