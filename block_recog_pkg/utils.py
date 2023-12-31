from rospkg import RosPack
from yaml import full_load
from os.path import join
from cv_bridge import CvBridge
from moveit_commander.conversions import list_to_pose
from numpy import pi
import rospy
import cv2
from tf_conversions import *
import numpy as np

pkg_path = RosPack().get_path("block_recog_pkg")
colors = full_load(open(join(pkg_path, "data/colors.yaml"), "rb"))
BLOCK_X = 0.075
BLOCK_Y = 0.025
BLOCK_Z = 0.015
bridge = CvBridge()
RGB_NODE = "/rgb/image_raw"
DEP_NODE = "/depth_to_rgb/image_raw"
RGB_PKL = "data/rgb.pkl"
DEP_PKL = "data/dep.pkl"
TOWER_MODEL = join(pkg_path,"data/jenga_tower_side_xy_m.stl")
TIME_LIMIT = rospy.Duration(10)
kern = cv2.getStructuringElement(cv2.MORPH_ERODE, (5,5))

import open3d as o3d

intrinsic = o3d.camera.PinholeCameraIntrinsic()
intrinsic.intrinsic_matrix = [
    [968.813, 0, 1023.83],
    [0, 968.635, 775.975],
    [0, 0, 1],
]
INIT_TF = toMatrix(fromMsg(list_to_pose([0, 0, -0.15, -pi/2, 0, pi/2])))

# INIT_TF = np.array([[0.7, -0.7, 0, 0], [0, 0, 1, 0], [-0.7, -0.7, 0, 0], [0, 0, 0, 1]])