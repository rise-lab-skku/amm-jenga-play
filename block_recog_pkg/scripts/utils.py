from rospkg import RosPack
from yaml import full_load
from os.path import join
from cv_bridge import CvBridge
from moveit_commander.conversions import list_to_pose
from numpy import pi
import rospy
import cv2

pkg_path = RosPack().get_path("block_recog_pkg")
colors = full_load(open(join(pkg_path, "data/colors.yaml"), "rb"))
BLOCK_X=0.075
BLOCK_Y=0.025
BLOCK_Z=0.015
bridge = CvBridge()
RGB_NODE="/rgb/image_raw"
DEP_NODE="/depth_to_rgb/image_raw"
RGB_PKL="data/rgb.pkl"
DEP_PKL="data/dep.pkl"
WORLD_LINK='panda_link0'
CAM_LINK='rgb_camera_link'
TIME_LIMIT=rospy.Duration(10)
kern=cv2.getStructuringElement(cv2.MORPH_RECT, (10,10))

JENGA_CAPTURE_POSE = list_to_pose([0.15, -0.5, 0.4, pi / 2, pi / 2, pi / 2])
DICE_CAPTURE_POSE = [0, 0, 0, 0, 0, 0]
RESTRICTED_FLOORS = 3
ESCAPE_JOINT = "panda_joint2"
ESCAPE_VALUE = -3*pi / 8