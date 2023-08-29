import rospy
import moveit_commander

import geometry_msgs.msg
import control_pkg
from math import pi, dist, cos
from copy import deepcopy as dcp
from block_recog_pkg.srv import (
    GetWorldCoord,
    GetWorldCoordRequest,
    CaptureImage,
    CaptureImageRequest,
    GetDiceColor,
    GetDiceColorRequest,
)

from cv_bridge import CvBridge

bridge = CvBridge()
import numpy as np


JENGA_CAPTURE_POSE = moveit_commander.conversions.list_to_pose([0, -0.5, 0.4, pi / 2, pi / 2, pi / 2])
DICE_CAPTURE_POSE = [0, 0, 0, 0, 0, 0]
RESTRICTED_FLOORS = 3
ESCAPE_JOINT = "panda_joint2"
ESCAPE_VALUE = -3*pi / 8


def all_close(goal, current, position_tolerance, orientation_tolerance):
    gl = moveit_commander.conversions.pose_to_list(goal)
    cl = moveit_commander.conversions.pose_to_list(current)

    d = dist(gl[:3], cl[:3])
    cos_phi_half = abs(sum(p * q for (p, q) in zip(gl[3:], cl[3:])))
    return d < position_tolerance and cos_phi_half > cos(orientation_tolerance / 2.0)


def initialize():
    moveit_commander.roscpp_initialize("")
    rospy.init_node("jenga_main", anonymous=True, disable_signals=True)

    robot = moveit_commander.RobotCommander()
    print(f"============ Root Link:  {robot.get_root_link():^20} ============")
    print(f"============ Move Groups:{str(robot.get_group_names()):^20} ============")

    gripper = control_pkg.gripper.Commander(fake=True)
    scene = control_pkg.scene.Commander()

    links = robot.get_link_names()
    grasp_link = links[list(map(lambda x: x.find("grasp") > -1, links)).index(True)]
    print(grasp_link)
    push_link = links[list(map(lambda x: x.find("push") > -1, links)).index(True)]
    manipulator = control_pkg.manipulator.Commander(gripper, grasp_link, push_link)

    return robot, gripper, scene, manipulator


robot, gripper, scene, manipulator = initialize()

# manipulator.ready()
# manipulator.set_joint_value_target(ESCAPE_JOINT, ESCAPE_VALUE)
# manipulator.plan_and_execute(None, None)
# manipulator.plan_and_execute(JENGA_CAPTURE_POSE, "grasper")
print("moved")
rospy.sleep(2)
input("dfsdfdsf")
############### take picture and callib ######################
rospy.wait_for_service("CaptureImage")
response = rospy.ServiceProxy("CaptureImage", CaptureImage).call(CaptureImageRequest())
if response.status == response.FAILED:
    rospy.logwarn("Failed to Capture Image")
elif response.status == response.SUCCESS:
    rospy.loginfo("Image Captured")
elif response.status == response.SKIPPED:
    rospy.loginfo("Image Capture Skipped")

# INIT 1
rospy.wait_for_service("GetWorldCoordinates")

response = rospy.ServiceProxy("GetWorldCoordinates", GetWorldCoord).call(GetWorldCoordRequest(target_block="init 1"))
coord0 = response.center
coord1 = response.tcp_target

# INIT 2

response = rospy.ServiceProxy("GetWorldCoordinates", GetWorldCoord).call(GetWorldCoordRequest(target_block="init 2"))

coord2 = response.center
coord3 = response.tcp_target

towermap = bridge.imgmsg_to_cv2(response.tower_map).astype("int").tolist()
print(towermap)
print([coord0, coord1, coord2, coord3])
input()
initial_towermap = towermap

scene.add_jenga([coord0, coord1, coord2, coord3])


def al(color, i):
    fl = towermap[i]
    if color in fl:
        idx = fl.index(color)
        if idx == 1:
            if fl[0] + fl[2] >= 0 and fl[0] * fl[2] >= 0:
                return i, 1
        else:
            if fl[1] >= 0:
                deep_sol = al(color, i + 1)
                if deep_sol is None:
                    return i, idx
                else:
                    if deep_sol[0] > i + 5:
                        return i, idx
                    return deep_sol
    if i < 11:
        return al(color, i + 1)
    else:
        return None


while True:
    # grasper.plan_and_execute(DICE_CAPTURE_POSE)
    response = rospy.ServiceProxy("GetDiceColor", GetDiceColor).call(GetDiceColorRequest())

    if response.success:
        dice_color = response.dice_color
    else:
        continue

    index = al(dice_color, RESTRICTED_FLOORS)
    command = "green 5"
    response = rospy.ServiceProxy("GetWorldCoordinates", GetWorldCoord).call(GetWorldCoordRequest(target_block=command))

    # vision result
    target_point = [
        dcp(response.center_x),
        dcp(response.center_y),
        dcp(response.center_z),
    ]
    temp_point = [
        dcp(response.target_x),
        dcp(response.target_y),
        dcp(response.target_z),
    ]
