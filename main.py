import rospy
import moveit_commander

import geometry_msgs.msg
import control_pkg
from moveit_msgs.msg import DisplayTrajectory as DT
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


JENGA_CAPTURE_POSE=[-0.05, -0.5, 0.5,pi / 2, pi / 2, pi / 2]
DICE_CAPTURE_POSE=[0,0,0,0,0,0]

def all_close(goal, current, position_tolerance, orientation_tolerance):
    gl = moveit_commander.conversions.pose_to_list(goal)
    cl = moveit_commander.conversions.pose_to_list(current)

    d = dist(gl[:3], cl[:3])
    cos_phi_half = abs(sum(p * q for (p, q) in zip(gl[3:], cl[3:])))
    return d < position_tolerance and cos_phi_half > cos(orientation_tolerance / 2.0)


def display_trajectory(plan, robot):
    disp_traj = DT()
    disp_traj.trajectory_start = robot.get_current_state()  # RobotState
    disp_traj.trajectory.append(plan)

    disp_traj_pub = rospy.Publisher("/move_group/display_planned_path", DT)
    disp_traj_pub.publish(disp_traj)


def initialize():
    moveit_commander.roscpp_initialize("")
    rospy.init_node("jenga_main", anonymous=True,disable_signals=True)

    robot = moveit_commander.RobotCommander()
    print(f"============ Root Link:  {robot.get_root_link():^20} ============")
    print(f"============ Move Groups:{str(robot.get_group_names()):^20} ============")

    move_group = moveit_commander.MoveGroupCommander("arm")
    gripper = control_pkg.gripper.Commander(fake=True)
    scene = control_pkg.scene.Commander()

    links = robot.get_link_names()
    grasp_link = links[list(map(lambda x: x.find("grasp") > -1, links)).index(True)]
    push_link = links[list(map(lambda x: x.find("push") > -1, links)).index(True)]
    grasper = control_pkg.manipulator.Commander(grasp_link, move_group,gripper)
    pusher = control_pkg.manipulator.Commander(push_link, move_group,gripper)

    return robot, move_group, gripper, scene, grasper, pusher


robot, move_group, gripper, scene, grasper, pusher = initialize()

# grasper.plan_and_execute(JENGA_CAPTURE_POSE)
# rospy.sleep(2)
############### take picture and callib ######################
rospy.wait_for_service("CaptureImage")
capture_image = rospy.ServiceProxy("CaptureImage", CaptureImage)
request_capture_image = CaptureImageRequest()

response = capture_image(request_capture_image)
if response.status == response.FAILED:
    rospy.logwarn("Failed to Capture Image")
elif response.status == response.SUCCESS:
    rospy.loginfo("Image Captured")
elif response.status == response.SKIPPED:
    rospy.loginfo("Image Capture Skipped")

# INIT 1
# rospy.wait_for_service("GetWorldCoordinates")

# get_coord = rospy.ServiceProxy("GetWorldCoordinates", GetWorldCoord)

# request = GetWorldCoordRequest()

# request.target_block = "init 1"

# response = get_coord(request)

# coord0 = response.center_coordinate
# coord1 = response.tcp_target_coordinate

# INIT 2
rospy.wait_for_service("GetWorldCoordinates")

get_coord = rospy.ServiceProxy("GetWorldCoordinates", GetWorldCoord)

request = GetWorldCoordRequest()

request.target_block = "init 2"

response = get_coord(request)

coord2 = response.center
coord3 = response.tcp_target

towermap_imgmsg = response.tower_map

towermap = bridge.imgmsg_to_cv2(towermap_imgmsg)
print(towermap)
input()
###############give me 4 poinst to make jenga################
points = [coord0,coord1,coord2,coord3]
# give me succeeding points

scene.add_jenga(points)

while True:
    get_dice_color = rospy.ServiceProxy("GetDiceColor", GetDiceColor)

    request_dice_color = GetDiceColorRequest()

    response_dice_color = get_dice_color(request_dice_color)

    if response_dice_color.success:
        dice_color = response_dice_color.dice_color
    else:
        continue
    
    # command = input("\njenga to extract: ")  # ex. "green 5"
    # if command == "q":
    #     break

    ################### get two points and method #################
    # color = 'green'
    # num = 5
    rospy.wait_for_service("GetWorldCoordinates")

    get_coord = rospy.ServiceProxy("GetWorldCoordinates", GetWorldCoord)

    request = GetWorldCoordRequest()

    request.target_block = command
    print(request)
    print("heejslfdjsdlkfjlk")
    response = get_coord(request)

    print(response.success)
    print(dcp(response.center_x))
    print(dcp(response.center_y))
    print(dcp(response.center_z))
    print(dcp(response.target_x))
    print(dcp(response.target_y))
    print(dcp(response.target_z))
    print(response.push)
    
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
