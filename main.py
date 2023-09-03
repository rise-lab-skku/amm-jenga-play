import rospy
import moveit_commander

import control_pkg
from math import pi
from copy import deepcopy as dcp
import numpy as np
import block_recog_pkg
import tf
from tf_conversions import *
import moveit_msgs
import open3d as o3d
import trajectory_msgs

DICE_GRASP_POSE = moveit_commander.conversions.list_to_pose(
    [0.4, -0.1, 0.4, pi, 0, -pi / 4]
)
DICE_ROLL_POSE = []
ROLL_JOINT = "panda_joint7"
DICE_WIDTH = 0.02
JENGA_CAPTURE_POSE = moveit_commander.conversions.list_to_pose(
    [0.15, -0.5, 0.4, pi / 2, pi / 2, pi / 2]
)
DICE_CAPTURE_POSE = [0, 0, 0, 0, 0, 0]
RESTRICTED_FLOORS = 3
ESCAPE_JOINT = "panda_joint2"
ESCAPE_VALUE = -3 * pi / 8

WORLD_LINK = "panda_link0"
CAM_LINK = "rgb_camera_link"


from moveit_msgs.msg import RobotTrajectory as RT
from trajectory_msgs.msg import JointTrajectoryPoint as JTP


def roll_dice():
    # goto pre-determined pose, init gripper, and then grasp dice and go back to ready pose
    manipulator.plan_and_execute(DICE_GRASP_POSE, None)
    gripper.homing()
    input("please put the dice.")
    gripper.grasp(DICE_WIDTH, 0.1, 1)
    manipulator.ready()

    # set first waypoint to ready pose
    rt = RT()
    rt.joint_trajectory.joint_names = [f"panda_joint{i}" for i in range(1, 8)]
    jp = list(
        manipulator.get_named_target_values("ready").values()
    )  # get named ~~ returns dictionary
    rt.joint_trajectory.points.append(JTP(positions=dcp(jp)))

    # set joint7's goal angle to +2rad, reach the goal in 2secs
    jp[-1] = 2
    rt.joint_trajectory.points.append(
        JTP(positions=dcp(jp), time_from_start=rospy.Duration(2))
    )

    gripper.move(2 * DICE_WIDTH, 0.1)
    print("roll dice done")


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
# roll_dice()
input()

# manipulator.ready()
# manipulator.set_joint_value_target(ESCAPE_JOINT, ESCAPE_VALUE)
# manipulator.plan_and_execute(None, None)
# manipulator.plan_and_execute(JENGA_CAPTURE_POSE, "grasper")

image = block_recog_pkg.image.Commander(fake=True)
image.capture()
colors = block_recog_pkg.image.colors  # need deepcopy?

total_points = 0
total_pcd = o3d.geometry.PointCloud()
for i, color in enumerate(colors):
    color["masks"] = image.get_masks(i)
    rospy.loginfo(
        f"Color: {color['name']}, recognized block bnmbers: {len(color['masks'])-1}"
    )

    color["pcd"] = image.get_pcd(color["masks"][-1])
    points = color["pcd"].points
    rospy.loginfo(f"Color: {color['name']}, PCD generated with {points} points")
    total_points += points
    total_pcd += color["pcd"]

input()

tower = block_recog_pkg.tower.Commander(total_points)
rospy.loginfo("Start ICP ...")
mat1 = tower.get_transform(total_pcd)

listener = tf.TransformListener()
mat2 = toMatrix(fromTf(listener.lookupTransform(WORLD_LINK, CAM_LINK, rospy.Time(0))))
world2tower = toTf(fromMatrix(np.inner(mat1, mat2)))
tf.TransformBroadcaster().sendTransform(
    world2tower[0], world2tower[1], rospy.Time(0), "mesh", WORLD_LINK
)

tower.build()
scene.add_jenga(tower.bases)


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