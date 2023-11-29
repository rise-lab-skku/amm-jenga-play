import rospy
import moveit_commander

import control_pkg
from math import pi
from copy import deepcopy as dcp
import numpy as np

import block_recog_pkg
from tf_conversions import *
import moveit_msgs
import open3d as o3d
import trajectory_msgs
from spatialmath import UnitQuaternion as UQ
from geometry_msgs.msg import Point
import tf2_ros


# DICE_GRASP_POSE = moveit_commander.conversions.list_to_pose([0.4, -0.1, 0.4, pi, 0, -pi / 4])
DICE_ROLL_POSE = [
    1.1604983166837157,
    -0.8248791238641395,
    -1.623791347626842,
    -2.4889099888591577,
    -1.0361005790516955,
    2.1055240211752397,
    0.7910710103587447,
]
ROLL_JOINT = "panda_joint7"
DICE_WIDTH = 0.02
JENGA_CAPTURE_POSE = moveit_commander.conversions.list_to_pose([0.17, -0.5, 0.4, pi / 2, pi / 2, pi / 2])
DICE_CAPTURE_POSE = [0, 0, 0, 0, 0, 0]
RESTRICTED_FLOORS = 3
ESCAPE_JOINT = "panda_joint2"
ESCAPE_VALUE = -3 * pi / 8

WORLD_LINK = "world"
CAM_LINK = "rgb_camera_link"


from moveit_msgs.msg import RobotTrajectory as RT
from trajectory_msgs.msg import JointTrajectoryPoint as JTP
from std_msgs.msg import Header


def roll_dice():
    # init gripper, and then grasp dice and go back to ready pose
    gripper.move(0.08, 0.05)
    gripper.wait()
    print("please put the dice.")
    rospy.sleep(2.5)
    gripper.grasp(DICE_WIDTH, 0.1, 1.5)
    gripper.wait()

    # set first waypoint to ready pose
    rt = RT()
    rt.joint_trajectory.joint_names = [f"panda_joint{i}" for i in range(1, 8)]
    jp = manipulator.get_current_joint_values()
    rt.joint_trajectory.points.append(JTP(positions=dcp(jp), velocities=[0] * 7))
    jp[-1] = 0
    rt.joint_trajectory.points.append(JTP(positions=dcp(jp), time_from_start=rospy.Duration(2), velocities=[0] * 7))
    manipulator.execute(rt)
    rospy.sleep(0.25)
    rt.joint_trajectory.points.pop(0)
    rt.joint_trajectory.points[0].time_from_start = rospy.Duration(0)
    jp[-1] = pi / 2
    rt.joint_trajectory.points.append(
        JTP(positions=dcp(jp), velocities=[0] * 6 + [7 * pi / 16], time_from_start=rospy.Duration(2.25))
    )
    jp[-1] = 3 * pi / 4
    rt.joint_trajectory.points.append(JTP(positions=dcp(jp), velocities=[0] * 7, time_from_start=rospy.Duration(3)))
    manipulator.execute(rt, wait=False)
    while manipulator.get_current_joint_values()[-1] < 3 * pi / 8:
        pass

    gripper.move(2 * DICE_WIDTH, 0.1)
    gripper.wait()
    rospy.sleep(0.5)

    rt.joint_trajectory.points = []
    jp = manipulator.get_current_joint_values()
    rt.joint_trajectory.points.append(JTP(positions=dcp(jp), velocities=[0] * 7))
    jp[-1] = pi / 4
    rt.joint_trajectory.points.append(JTP(positions=dcp(jp), velocities=[0] * 7, time_from_start=rospy.Duration(3)))
    manipulator.execute(rt)
    rospy.sleep(2)
    print("roll dice done")


moveit_commander.roscpp_initialize("")
rospy.init_node("jenga_main", anonymous=True, disable_signals=True)

robot = moveit_commander.RobotCommander()
print(f"============ Root Link:  {robot.get_root_link():^20} ============")
print(f"============ Move Groups:{str(robot.get_group_names()):^20} ============")

gripper = control_pkg.gripper.Commander()
print("gripper ready")
scene = control_pkg.scene.Commander()
print("scene ready")

links = robot.get_link_names()
grasp_link = links[list(map(lambda x: x.find("grasp") > -1, links)).index(True)]
push_link = links[list(map(lambda x: x.find("push") > -1, links)).index(True)]
manipulator = control_pkg.manipulator.Commander(grasp_link, push_link)

# manipulator.ready()
# manipulator.set_joint_value_target(ESCAPE_JOINT, ESCAPE_VALUE)
# manipulator.plan_and_execute(None, None)
# manipulator.plan_and_execute(JENGA_CAPTURE_POSE, "grasper")
rospy.sleep(3)
image = block_recog_pkg.image.Commander()
image.capture()
colors = block_recog_pkg.image.colors  # need deepcopy?

total_points = 0
total_pcd = o3d.geometry.PointCloud()
for i, color in enumerate(colors):
    color["masks"] = image.get_masks(i)
    rospy.loginfo(f"Color: {color['name']}, recognized {len(color['masks']) - 1} blocks")

    color["pcd"] = image.get_pcd(color["masks"][-1])
    points = color["pcd"].points
    rospy.loginfo(f"Color: {color['name']}, PCD generated with {len(points)} points")
    total_points += len(points)
    total_pcd += color["pcd"]


tower = block_recog_pkg.tower.Commander(block_recog_pkg.utils.TOWER_MODEL, len(total_pcd.points))
rospy.loginfo("Start ICP...")
mat1 = tower.get_transform(total_pcd)
rospy.loginfo("ICP done")
print(mat1)


rate = rospy.Rate(10)
listener = tf2_ros.Buffer()
tf2_ros.TransformListener(listener)
while not rospy.is_shutdown():
    try:
        tf = listener.lookup_transform(WORLD_LINK, CAM_LINK, rospy.Time()).transform
        break
    except:
        rate.sleep()
        continue

mat2 = toMatrix(fromMsg(Pose(tf.translation, tf.rotation)))
print(mat2)
trans=mat1[:3,3]+mat2[:3,3]
rot=np.inner(mat2[:3,:3],mat1[:3,:3])
mat=np.vstack((np.hstack((rot,trans.reshape(3,1))),(0,0,0,1)))
print(mat)

broadcaster = tf2_ros.StaticTransformBroadcaster()
from geometry_msgs.msg import TransformStamped

ts = TransformStamped(header=Header(frame_id=WORLD_LINK), child_frame_id="mesh")
frame = fromMatrix(mat)

msg = toMsg(fromTf((frame.p, UQ(frame.M.GetQuaternion()).A)))
ts.transform.translation = msg.position
ts.transform.rotation = msg.orientation

rospy.loginfo("trying to broadcast")
t0 = rospy.Time.now()
while not listener.can_transform("mesh", "world", rospy.Time()):
    rate.sleep()
    if rospy.Time.now() - t0 > rospy.Duration(5):
        rospy.logwarn("broadcast failed")
        break
    broadcaster.sendTransform(ts)

print(listener.lookup_transform('mesh','world',rospy.Time()))
tower.build(listener)
scene.add_jenga(tower.bases)
print(tower.origin)
print(tower.bases)
input("done")


dice = block_recog_pkg.image.Commander()
while True:
    input("press enter to roll dice")
    roll_dice()
    dice.clear()
    dice.capture()
    dominant_color = None
    domintant_area = 0

    for i in range(len(colors)):
        masks = dice.get_masks(i)
        area = dice.calculate_area((masks[-1] * 255).astype(np.uint8))
        if area > domintant_area:
            domintant_area = area
            dominant_color = i

    print(f"dominant color: {colors[dominant_color]['name']}")
    print(f"dominant area: {domintant_area}")


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
