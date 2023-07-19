from __future__ import print_function # from six.moves import input
import sys
import os
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi, tau, dist, fabs, cos, atan, sin, sqrt
from std_msgs.msg import String
import franka_gripper.msg
import actionlib
from moveit_commander.conversions import pose_to_list
import time
from moveit_commander import RobotCommander, PlanningSceneInterface
from copy import deepcopy as dcp
import trajectory_msgs.msg
from visualization_msgs.msg import Marker

from block_recog_pkg.srv import GetWorldCoord, GetWorldCoordRequest, GetWorldCoordResponse, CaptureImage, CaptureImageRequest, CaptureImageResponse

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    norm = np.sqrt(qx**2 + qy**2+qz**2+qw**2)
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    return qx, qy, qz, qw

class MoveitPython(object):
    def __init__(self):
        super(MoveitPython, self).__init__()

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object()
        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to a planning group (group of joints).  In this tutorial the group is the primary
        # arm joints in the Panda robot, so we set the group's name to "panda_arm".
        # If you are using a different robot, change this value to the name of your robot
        # arm planning group.
        # This interface can be used to plan and execute motions:
        #group_name = "panda_arm" to make TCP as Panda_manipulaotor new fingert tip
        self.group_name = "panda_manipulator"
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        #print(dir(move_group))
        #print(move_group.get_goal_joint_tolerance())
        #print(move_group.get_planning_time())
        #move_group.set_goal_tolerance(0.001)
        move_group.set_goal_joint_tolerance(0.001)
        move_group.set_goal_position_tolerance(0.001)
        move_group.set_goal_orientation_tolerance(0.001)
        move_group.set_planning_time(20.0)
        print("setting the goal tolerance(joint, position, orientation). now tolerance", move_group.get_goal_tolerance())
        print("setting the planning time. Default 5s, now changed as",move_group.get_planning_time())
        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # END_SUB_TUTORIAL

        # BEGIN_SUB_TUTORIAL basic_info
        #
        # Getting Basic Information
        # ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        #print(robot.get_current_state())
        print("")
        # END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def change_tcp(self, method=None, group_name=None):
        robot = moveit_commander.RobotCommander()
        #print("hello",self.group_name)

        if method:
            method = "push"
        else:
            method = "grasp"
            
        if method is None:
            if group_name is None:
                if self.group_name == "panda_manipulator2":
                    self.group_name = "panda_manipulator"
                else:
                    self.group_name = "panda_manipulator2"
            else:
                self.group_name = group_name
        elif method == "grasp":
            self.group_name = "panda_manipulator"
        elif method == "push":
            self.group_name = "panda_manipulator2"
        else:
            print("define method correctly")
            self.group_name = "panda_manipulator2"

        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        print(self.group_name)
        #print(dir(move_group))
        #print(move_group.get_goal_joint_tolerance())
        #print(move_group.get_planning_time())
        #move_group.set_goal_tolerance(0.001)
        move_group.set_goal_joint_tolerance(0.001)
        move_group.set_goal_position_tolerance(0.001)
        move_group.set_goal_orientation_tolerance(0.001)
        move_group.set_planning_time(20.0)
        print("setting the goal tolerance(joint, position, orientation). now tolerance", move_group.get_goal_tolerance())
        print("setting the planning time. Default 5s, now changed as",move_group.get_planning_time())
        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        #print(robot.get_current_state())
        print("")
        # END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def rpy_goal(self, rpy, xyz):
        move_group = self.move_group
        end_effetor = move_group.get_end_effector_link()
        print(end_effetor)
        print(move_group.get_current_rpy(end_effector_link=end_effetor))
        move_group.set_position_target(xyz, end_effetor)
        #pose_goal = geometry_msgs.msg.Pose()
        pose_goal = [xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]]
        #print(type(pose_goal))
        pose_goal = move_group.set_pose_target(pose_goal, end_effetor)
        #move_group.go(plan, wait=True)

        is_success, traj, planning_time, error_code = move_group.plan(joints=pose_goal)
        _msg = "success" if is_success else "failed"
        rospy.loginfo(f"Planning is [ {_msg} ] (error code : {error_code.val}, planning time : {planning_time:.2f}s)")
        self.display_trajectory(traj)

        input("\nWait for Enter to execute the plan...")
        if is_success:
            rospy.loginfo("Executing the plan")
            move_group.execute(traj, wait=True)

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        print("Current pose state", current_pose)
        return all_close(pose_goal, current_pose, 0.001)

    def two_points_to_rpy(self, target_point, temp_point):
        dx = target_point[0] - temp_point[0]
        dy = target_point[1] - temp_point[1]
        if abs(dx) <= 0.0001:
            direction = 0
        else:
            direction = atan(dy/dx)-pi/2
        print(direction)
        rpy = [pi/2,pi/2,direction]
        return rpy

    def go_to_default(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        print("Current joint state", joint_goal)
        joint_goal = [0, -tau / 8, 0, -3*tau / 8, 0, tau / 4, tau / 8] # Default joint angle in radian
        print("Goal joint state", joint_goal)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        # move_group.go(joint_goal, wait=True)

        # The fallowing codes are same with move_group.go()
        #   - success flag : boolean
        #   - trajectory message : RobotTrajectory
        #   - planning time : float
        #   - error code : MoveitErrorCodes
        is_success, traj, planning_time, error_code = move_group.plan(joints=joint_goal)
        _msg = "success" if is_success else "failed"
        rospy.loginfo(f"Planning is [ {_msg} ] (error code : {error_code.val}, planning time : {planning_time:.2f}s)")
        self.display_trajectory(traj)

        input("\nWait for any key to execute the plan...")
        if is_success:
            rospy.loginfo("Executing the plan")
            move_group.execute(traj, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.001)

    def plan_cartesian_path(self, cartesian_move, avoid_collisions=False):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        # BEGIN_SUB_TUTORIAL plan_cartesian_path
        #
        # Cartesian Paths
        # ^^^^^^^^^^^^^^^
        # You can plan a Caprint(move_client(width=width))
        waypoints = []        # print("============ Startiing Position Move ============")
        # orientation = get_quaternion_from_euler(pi/2,pi/4,pi/2)
        # print("target orientation is", orientation)
        # position = [-0.1,0.25, 0.4]
        # move.go_to_pose_goal(orientation, position)


        wpose = move_group.get_current_pose().pose
        wpose.position.x += cartesian_move[0]
        wpose.position.y += cartesian_move[1]
        wpose.position.z += cartesian_move[2]
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0, avoid_collisions  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
        # END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # BEGIN_SUB_TUTORIAL display_trajectory
        #
        # Displaying a Trajectory
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        #
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        # END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        # BEGIN_SUB_TUTORIAL execute_plan
        #
        # Executing a Plan
        # ^^^^^^^^^^^^^^^^
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        move_group.execute(plan, wait=True)

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        # END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        # BEGIN_SUB_TUTORIAL wait_for_scene_update
        #
        # Ensuring Collision Updates Are Received
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        # or dies before actually publishing the scene update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        # For the purpose of this tutorial, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed.
        # To avoid waiting for scene updates like this at all, initialize the
        # planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attachios.system("python _test_ex2.py")ng the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        # END_SUB_TUTORIAL

    def jenga_extract(self, target_point, temp_point, method):

        self.change_tcp(method=method)
        time.sleep(1)
        rpy = self.two_points_to_rpy(target_point=target_point,temp_point=temp_point)
        x = temp_point[0] - target_point[0]
        y = temp_point[1] - target_point[1]
        r = sqrt(x **2 + y **2)
        if method is False:# grasp
            self.rpy_goal(rpy=rpy,xyz=target_point)
            print("rpy", rpy)
            grasp_client(0.07)
            time.sleep(1)
            extract = [0.11*x/r, 0.11* y/r,0]
            self.execute_plan(self.plan_cartesian_path(cartesian_move=extract, avoid_collisions=False)[0])
            time.sleep(1)
            move_client(0.08)
            print("grasp extraction complete")

        elif method is True:# "push"
            self.rpy_goal(rpy=rpy,xyz=temp_point)
            print("rpy", rpy)
            move_client(0)
            time.sleep(1)
            extract = [-0.10*x/r, -0.10* y/r,0]
            self.execute_plan(self.plan_cartesian_path(cartesian_move=extract, avoid_collisions=False)[0])
            time.sleep(1)
            extract = [0.11*x/r, 0.11* y/r,0]
            self.execute_plan(self.plan_cartesian_path(cartesian_move=extract, avoid_collisions=False)[0])
            time.sleep(1)
            move_client(0.08)
            print("push extraction complete")
        else:
            print("wrong method")


    # collsion control
    def add_camera_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene

        # Adding Objects to the Planning Scene
        # Camera attched to end effector. We have to avoid collision with the camera
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand" # attaching to the end effector
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.08  # above the panda_hand frame
        box_pose.pose.position.y = 0.0  # above the panda_hand frame
        box_pose.pose.position.z = 0.02  # above the panda_hand frame
        box_name = "camera_box"
        scene.add_box(box_name, box_pose, size=(0.04, 0.1, 0.1))
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_finger_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        # First, we will create a box in the planning scene between the fingers:
        # long finger tips attched to gripper finger.. We have to avoid collision with the finger tips
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand" # attaching to the end effector
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.16  # above the panda_hand frame
        box_name = "finger_box"
        scene.add_box(box_name, box_pose, size=(0.02, 0.09, 0.09))
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_finger_boxes(self, timeout=4):
        box1_name = self.box_name
        box2_name = self.box_name
        scene = self.scene
        # First, we will create a box in the planning scene between the fingers:
        # long finger tips attched to gripper finger.. We have to avoid collision with the finger tips
        box1_pose = geometry_msgs.msg.PoseStamped()
        box1_pose.header.frame_id = "panda_hand" # attaching to the end effector
        box1_pose.pose.orientation.w = 1.0
        box1_pose.pose.position.z = 0.16  # above the panda_hand frame
        box1_pose.pose.position.y = 0.05
        box2_pose = geometry_msgs.msg.PoseStamped()
        box2_pose.header.frame_id = "panda_hand" # attaching to the end effector
        box2_pose.pose.orientation.w = 1.0
        box2_pose.pose.position.z = 0.16  # above the panda_hand frame
        box2_pose.pose.position.y = -0.05
        box1_name = "finger_box1"
        box2_name = "finger_box2"
        scene.add_box(box1_name, box1_pose, size=(0.01, 0.01, 0.09))
        scene.add_box(box2_name, box2_pose, size=(0.01, 0.01, 0.09))
        self.box_name = box1_name
        self.box_name = box2_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_jenga_box(self, points = None, timeout=4):
        box_name = self.box_name
        scene = self.scene
        # Add jenga box to the scene.
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        if points is None:
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.x = 0.4
            box_pose.pose.position.y = -0.4
            box_pose.pose.position.z = 0.165
        else:
            box_pose.pose.position.x = (points[0][0]+points[1][0]+points[2][0]+points[3][0])/4
            box_pose.pose.position.y = (points[0][1]+points[1][1]+points[2][1]+points[3][1])/4
            box_pose.pose.position.z = 0.165
            rpy = self.two_points_to_rpy(points[0],points[1])
            print(rpy)
            x, y, z, w = get_quaternion_from_euler(0,0,rpy[2])
            box_pose.pose.orientation.x = x
            box_pose.pose.orientation.y = y
            box_pose.pose.orientation.z = z
            box_pose.pose.orientation.w = w
        box_name = "jenga_box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.33))
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_camera_box(self, timeout=4):
        # attching camera box to panda hand
        box_name = "camera_box"
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def attach_finger_boxes(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box1_name = "finger_box1"
        box2_name = "finger_box2"
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box1_name, touch_links=touch_links)
        scene.attach_box(eef_link, box2_name, touch_links=touch_links)
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_finger_box(self, timeout=4):
        box_name = "finger_box"
        scene = self.scene
        eef_link = self.eef_link
        # We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_finger_box(self, timeout=4):
        box_name = "finger_box"
        scene = self.scene

        # Removing Objects from the Planning Scene
        # We can remove the box from the world.
        scene.remove_world_object(box_name)
        # **Note:** The object must be detached before we can remove it from the world
        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

    #not usnig now.
    def go_to_back_position(self):
        orientation = [pi/2,pi/2,pi/2]
        position = [0.34,-0.4,0.2]
        self.rpy_goal(orientation, position)
        return True

    def go_to_right_position(self):
        orientation = [0,pi/2,-pi/2]
        position = [0.4,-0.34,0.2]
        self.rpy_goal(orientation, position)
        return True

# gripper control
def grasp_client(width=0.022):
    # Creates the SimpleActionClient, passing the type of the action
    # (GraspAction) to the constructor.
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.GraspGoal()
    goal.width = width
    goal.epsilon.inner = 0.007
    goal.epsilon.outer = 0.0085
    goal.speed = 0.1
    goal.force = 5

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GraspResult

def move_client(width=0.022):
    # Creates the SimpleActionClient, passing the type of the action
    # (GraspAction) to the constructor.
    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.MoveGoal()
    goal.width = width
    goal.speed = 0.1

    # Sends the goal to the action server.
    client.send_goal(goal)
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GraspResult



def main():

    try:
        move = MoveitPython()
        time.sleep(1)

        #os.system("python3 "+os.path.dirname(__file__)+"/jenga_obstacle_environment.py")
        rospy.sleep(1)
        # print("============ Default Jenga Playing Scene ============")
        # print("============ Adding Collision control objects ============")
        # move.add_camera_box()
        # move.add_finger_boxes()
        # move.attach_camera_box()
        # move.attach_finger_boxes()
        # print("============ Adding Jenga to the scene ============")
        # move.add_jenga_box()

        while True:
            command=input("\ncommand:")

            #planning and execution functions
            if command=="init":
                move.go_to_default()
                move_client()
                move_client(0.08)
            elif command == "quit":
                break
            elif command == "change tcp":
                move.change_tcp(method=None, group_name=None)
            elif command == "cartesian":
                ans=input("cartesian move:")
                cartesian_move = [float(ans.split(' ')[i]) for i in range(3)]
                cartesian_plan, fraction = move.plan_cartesian_path(cartesian_move, avoid_collisions=False)
                move.display_trajectory(cartesian_plan)
                if input("if the plan is valid press enter to execute or press q to abort") =='q':
                    print("quit")
                    continue
                move.execute_plan(cartesian_plan)
            elif command == "goal":
                tmp=input("Orientation_RPY:").split(' ')
                rpy = [float(tmp[0]), float(tmp[1]), float(tmp[2])]
                tmp=input("Position:").split(' ')
                xyz = [float(tmp[0]), float(tmp[1]), float(tmp[2])]
                print(rpy)
                print(xyz)
                move.rpy_goal(rpy, xyz)
            elif command =="rpy2":
                rpy = move.two_points_to_rpy(target_point=[0.3,-0.4,0.4],temp_point=[0.6, -0.5,0.21])
                move.rpy_goal(rpy,[0.3,-0.4,0.4])



            #gripper functions
            elif command=="grasp":
                width = float(input("grasp width:"))
                print("target gripping width is", width, "grip success status: ",move.grasp_client(width=width))
            elif command=="move":
                width = float(input("move width:"))
                print("target gripper width is", move.move_client(width=width))

            #jenga playing functions
            elif command=="back":
                move.go_to_back_position()
            elif command=="right":
                move.go_to_right_position()

            elif command=="test":
                #move.move_client(0.08)
                # move.go_to_default()
                # time.sleep(1)

                # #camera position
                move.rpy_goal([pi/2, pi/2, pi/2],[0., -0.4, 0.2])

                ############### take picture and callib ######################
                rospy.wait_for_service('CaptureImage')
                capture_image = rospy.ServiceProxy('CaptureImage', CaptureImage)
                request_capture_image = CaptureImageRequest()

                response = capture_image(request_capture_image)

                if response.status == response.FAILED:
                    rospy.logwarn("Failed to Capture Image")
                elif response.status == response.SUCCESS:
                    rospy.loginfo("Image Captured")
                elif response.status == response.SKIPPED:
                    rospy.loginfo("Image Capture Skipped")

                # INIT 1
                rospy.wait_for_service('GetWorldCoordinates')

                get_coord = rospy.ServiceProxy('GetWorldCoordinates', GetWorldCoord)

                request = GetWorldCoordRequest()

                request.target_block = "init 1"

                response = get_coord(request.target_block)

                jenga_coordinate1_x = copy.deepcopy(copy.deepcopy(response.center_x))
                jenga_coordinate1_y = copy.deepcopy(copy.deepcopy(response.center_y))
                jenga_coordinate1_z = copy.deepcopy(copy.deepcopy(response.center_z))
                jenga_coordinate2_x = copy.deepcopy(response.target_x)
                jenga_coordinate2_y = copy.deepcopy(response.target_y)
                jenga_coordinate2_z = copy.deepcopy(response.target_z)
                # print(copy.deepcopy(response.center_y))
                # print(copy.deepcopy(response.center_z))
                # print(copy.deepcopy(response.target_x))
                # print(copy.deepcopy(response.target_y))
                # print(copy.deepcopy(response.target_z))

                # INIT 2
                rospy.wait_for_service('GetWorldCoordinates')

                get_coord = rospy.ServiceProxy('GetWorldCoordinates', GetWorldCoord)

                request = GetWorldCoordRequest()

                request.target_block = "init 2"

                response = get_coord(request.target_block)

                jenga_coordinate3_x = copy.deepcopy(response.center_x)
                jenga_coordinate3_y = copy.deepcopy(response.center_y)
                jenga_coordinate3_z = copy.deepcopy(response.center_z)
                jenga_coordinate4_x = copy.deepcopy(response.target_x)
                jenga_coordinate4_y = copy.deepcopy(response.target_y)
                jenga_coordinate4_z = copy.deepcopy(response.target_z)

                # rospy.loginfo(jenga_coordinate1_x)
                # rospy.loginfo(jenga_coordinate1_y)
                # rospy.loginfo(jenga_coordinate1_z)

                # marker로 좌표 check
                # marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
                # marker = Marker()

                # marker.header.frame_id = "panda_link0"
                # marker.header.stamp = rospy.Time.now()
                # marker.type = marker.SPHERE
                # marker.action = marker.ADD
                # marker.scale.x = 0.01
                # marker.scale.y = 0.01
                # marker.scale.z = 0.01
                # marker.color.a = 1.0
                # marker.color.r = 1.0
                # marker.color.g = 0.0
                # marker.color.b = 0.0
                # marker.pose.position.x = jenga_coordinate1_x
                # marker.pose.position.y = jenga_coordinate1_y
                # marker.pose.position.z = jenga_coordinate1_z
                # marker_pub.publish(marker)
                # rospy.sleep(1)

                rospy.loginfo(jenga_coordinate1_x)
                rospy.loginfo(jenga_coordinate1_y)
                rospy.loginfo(jenga_coordinate1_z)

                rospy.loginfo(jenga_coordinate2_x)
                rospy.loginfo(jenga_coordinate2_y)
                rospy.loginfo(jenga_coordinate2_z)

                rospy.loginfo(jenga_coordinate3_x)
                rospy.loginfo(jenga_coordinate3_y)
                rospy.loginfo(jenga_coordinate3_z)

                rospy.loginfo(jenga_coordinate4_x)
                rospy.loginfo(jenga_coordinate4_y)
                rospy.loginfo(jenga_coordinate4_z)




                ###############give me 4 poinst to make jenga################
                points = [[jenga_coordinate1_x, jenga_coordinate1_y, jenga_coordinate1_z],[jenga_coordinate2_x, jenga_coordinate2_y, jenga_coordinate2_z],[jenga_coordinate3_x, jenga_coordinate3_y, jenga_coordinate3_z],[jenga_coordinate4_x, jenga_coordinate4_y, jenga_coordinate4_z]]
                # give me succeeding points
                move.add_jenga_box(points)

                while True:
                    command = input("\njenga to extract: ")    # ex. "green 5"
                    if command == "q":
                        break

                    ################### get two points and method #################
                    # color = 'green'
                    # num = 5
                    rospy.wait_for_service('GetWorldCoordinates')

                    get_coord = rospy.ServiceProxy('GetWorldCoordinates', GetWorldCoord)

                    request = GetWorldCoordRequest()

                    request.target_block = command
                    print(request)
                    print('heejslfdjsdlkfjlk')
                    response = get_coord(request)

                    print(response.success)
                    print(copy.deepcopy(response.center_x))
                    print(copy.deepcopy(response.center_y))
                    print(copy.deepcopy(response.center_z))
                    print(copy.deepcopy(response.target_x))
                    print(copy.deepcopy(response.target_y))
                    print(copy.deepcopy(response.target_z))
                    print(response.push)

                    # copy.deepcopy(response.success)
                    # (copy.deepcopy(response.center_x))
                    # (copy.deepcopy(response.center_y))
                    # (copy.deepcopy(response.center_z))
                    # (copy.deepcopy(response.target_x))
                    # (copy.deepcopy(response.target_y))
                    # (copy.deepcopy(response.target_z))
                    # copy.deepcopy(response.push)





                    # vision result
                    target_point = [copy.deepcopy(response.center_x),copy.deepcopy(response.center_y),copy.deepcopy(response.center_z)]
                    temp_point = [copy.deepcopy(response.target_x),copy.deepcopy(response.target_y),copy.deepcopy(response.target_z)]
                    method = response.push #true-push fasle-pull
                    ###############################################################
                    move.jenga_extract(target_point,temp_point,method)

                time.sleep(1)
                #move.move_client(0.08)
                move.go_to_default()


            else:
                print("command error")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
