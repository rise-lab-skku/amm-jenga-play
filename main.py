import rospy
import moveit_commander

import control_pkg
from math import pi
from copy import deepcopy as dcp
# from block_recog_pkg.srv import (
#     GetWorldCoord,
#     GetWorldCoordRequest,
#     GetDiceColor,
#     GetDiceColorRequest,
# )

import numpy as np
import block_recog_pkg
import tf
from tf_conversions import *
import moveit_msgs
import trajectory_msgs

DICE_GRASP_POSE = moveit_commander.conversions.list_to_pose([0.4, -0.1, 0.4, pi, 0, -pi/4])
DICE_ROLL_POSE = []
ROLL_JOINT = "panda_joint7"
DICE_WIDTH = 0.02
JENGA_CAPTURE_POSE = moveit_commander.conversions.list_to_pose([0.15, -0.5, 0.4, pi / 2, pi / 2, pi / 2])
DICE_CAPTURE_POSE = [0, 0, 0, 0, 0, 0]
RESTRICTED_FLOORS = 3
ESCAPE_JOINT = "panda_joint2"
ESCAPE_VALUE = -3 * pi / 8
import threading
import time

def roll_dice():
    # manipulator.plan_and_execute(DICE_GRASP_POSE, None)
    gripper.homing()
    print("please put the dice.")
    rospy.sleep(3)
    gripper.grasp(DICE_WIDTH, 0.1, 1)
    # manipulator.ready()

    rt=moveit_msgs.msg.RobotTrajectory()
    rt.joint_trajectory.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
    jp = list(manipulator.get_current_joint_values())
    rt.joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint(positions=dcp(jp)))
    jp[-1]=2
    rt.joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint(positions=dcp(jp), time_from_start=rospy.Duration(2)))
    t=threading.Thread(target=manipulator.execute,args=[rt])
    t.start()
    time.sleep(1)
    gripper.move(2 * DICE_WIDTH, 0.1)
    t.join()
    print('roll dice done')


def initialize():
    moveit_commander.roscpp_initialize("")
    rospy.init_node("jenga_main", anonymous=True, disable_signals=True)

    robot = moveit_commander.RobotCommander()
    print(f"============ Root Link:  {robot.get_root_link():^20} ============")
    print(f"============ Move Groups:{str(robot.get_group_names()):^20} ============")

    gripper = control_pkg.gripper.Commander(eps=0.01)
    scene = control_pkg.scene.Commander()

    links = robot.get_link_names()
    grasp_link = links[list(map(lambda x: x.find("grasp") > -1, links)).index(True)]
    print(grasp_link)
    push_link = links[list(map(lambda x: x.find("push") > -1, links)).index(True)]
    manipulator = control_pkg.manipulator.Commander(gripper, grasp_link, push_link)

    return robot, gripper, scene, manipulator


robot, gripper, scene, manipulator = initialize()
print(manipulator.get_current_pose())
# manipulator.ready()
roll_dice()
input()

# manipulator.ready()
# manipulator.set_joint_value_target(ESCAPE_JOINT, ESCAPE_VALUE)
# manipulator.plan_and_execute(None, None)
# manipulator.plan_and_execute(JENGA_CAPTURE_POSE, "grasper")

image = block_recog_pkg.image.Commander()
input()
image.capture()
colors = block_recog_pkg.image.colors  # need deepcopy?

for i, color in enumerate(colors):
    color["masks"] = image.get_masks(i)
    rospy.loginfo(f"Color: {color['name']}, Recognized Block Numbers: {len(color['masks'])-1}")
    color["pcds"] = image.get_pcds(color["masks"])
    rospy.loginfo(f"Color: {color['name']}, pcd generated")

input()
tower = block_recog_pkg.tower.Commander(color["pcds"][-1] for color in colors)
rospy.loginfo("Start ICP ...")

listener = tf.TransformListener()
mat2 = toMatrix(fromTf(listener.lookupTransform(WORLD_LINK, CAM_LINK, rospy.Time(0))))
mat1 = tower.get_transform()
world2tower = toTf(fromMatrix(np.inner(mat1, mat2)))
tf.TransformBroadcaster().sendTransform(world2tower[0], world2tower[1], rospy.Time(0), "mesh", WORLD_LINK)

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
