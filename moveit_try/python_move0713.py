from __future__ import print_function # from six.moves import input
import sys
import os
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
import franka_gripper.msg
import actionlib
from moveit_commander.conversions import pose_to_list
import time
from moveit_commander import RobotCommander, PlanningSceneInterface

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


class MoveitPython(object):
    """MoveitPython"""

    def __init__(self):
        super(MoveitPython, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        #group_name = "panda_arm"
        group_name = "panda_manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
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
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        print("Current joint state", joint_goal)
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -3*tau / 8
        joint_goal[4] = 0
        joint_goal[5] = tau / 4  # 1/6 of a turn
        joint_goal[6] = tau / 8
        print("Goal joint state", joint_goal)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        print("hello Current joint state", current_joints)
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_right_state(self):

        move_group = self.move_group

        #print("Current joint state", joint_goal)
        joint_goal = [-0.4167545667813553, 0.6317899457437121, 0.964818479466587, -2.316581751144892, -1.3126282230777986, 0.9567712269792334, 0.5524828658164344]
        print("Goal joint state", joint_goal)
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_pose = self.move_group.get_current_pose().pose
        current_joints = move_group.get_current_joint_values()
        print("Current pose state", current_pose)
        print("Current joint state", current_joints)
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_back_state(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        print("Current joint state", joint_goal)
        joint_goal = [-0.23081663022630322, -1.5116293250424995, -1.8860979072420616, -2.6022039394421976, -0.024275083909371775, 2.363655958881634, 2.0549816900089564]
        print("Goal joint state", joint_goal)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movementin
        move_group.stop()

        ## END_SUB_TUTORIAL
        # For testing:
        current_pose = self.move_group.get_current_pose().pose
        current_joints = move_group.get_current_joint_values()
        print("Current pose state", current_pose)
        print("Current joint state", current_joints)
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, orientation, position):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = orientation[3]
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]

        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)


        #plan = move_group.plan()



        #return plan


        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        current_joint = move_group.get_current_joint_values()
        print("Current pose state", current_pose)
        print("Current joint state", current_joint)
        return all_close(pose_goal, current_pose, 0.01)

    def go_to_default(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        print("Current joint state", joint_goal)
        joint_goal = [0, -tau / 8, 0, -3*tau / 8, 0, tau / 4, tau / 8]
        print("Goal joint state", joint_goal)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL
        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def plan_cartesian_path(self, cartesian_move):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Caprint(move_client(width=width))
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
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
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
        ## END_SUB_TUTORIAL


    def add_tcp_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.16  # above the panda_hand frame
        box_name = "tcp_box"
        scene.add_box(box_name, box_pose, size=(0.02, 0.09, 0.085))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_camera_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.08  # above the panda_hand frame
        box_pose.pose.position.y = 0.0  # above the panda_hand frame
        box_pose.pose.position.z = 0.05  # above the panda_hand frame
        box_name = "camera_box"
        scene.add_box(box_name, box_pose, size=(0.04, 0.1, 0.1))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )


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
    goal.epsilon.inner = 0.008
    goal.epsilon.outer = 0.008
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
    return [qx, qy, qz, qw]

def main():

    try:
        move = MoveitPython()
        rospy.sleep(1)
        #print("============ Default state ============")
        move.add_tcp_box()
        move.add_camera_box()
        move.attach_box()


        while True:
            command=input("\ncommand:")
            if command=="init":
                move_client()
                move_client(0.08)
                move.go_to_default()
            if command=="init2":
                move.go_to_joint_state()
            elif command=="goal":
                ans=input("orientation_euler:").split(' ')
                orientation = get_quaternion_from_euler(float(ans[0]), float(ans[1]), float(ans[2]))
                ans=input("position:")
                position = [float(ans.split(' ')[i]) for i in range(3)]
                move.go_to_pose_goal(orientation, position)
            elif command == "cartesian":
                ans=input("cartesian move:")
                cartesian_move = [float(ans.split(' ')[i]) for i in range(3)]
                cartesian_plan, fraction = move.plan_cartesian_path(cartesian_move)
                move.display_trajectory(cartesian_plan)
                if input("plan ok enter enter")=='q':
                    print("quit")
                    continue
                move.execute_plan(cartesian_plan)
            elif command=="grasp":
                width = float(input("grasp width:"))
                print(grasp_client(width=width))
            elif command=="move":
                width = float(input("move width:"))
                print(move_client(width=width))


            elif command=="back":
                move.go_to_back_state()
            elif command=="right":
                move.go_to_right_state()



            elif command=="cue_test":
                move.execute_plan(move.plan_cartesian_path([0.11,0,0])[0])
                time.sleep(1)
                grasp_client(0.072)
                time.sleep(1)
                move.execute_plan(move.plan_cartesian_path([-0.11,0,0])[0])
                time.sleep(1)
                move_client(0.08)
                time.sleep(1)
                move.execute_plan(move.plan_cartesian_path([0,0,0.015])[0])
                time.sleep(1)
                move_client(0)
                time.sleep(1)
                move.execute_plan(move.plan_cartesian_path([0.085,0,0])[0])
                time.sleep(1)
                move.execute_plan(move.plan_cartesian_path([-0.085,0,0])[0])
                time.sleep(1)
                move_client(0.08)


            else:
                print("command error")
        #move.go_to_joint_state()


        #
        #move.display_trajectory(plan)
        #move.execute_plan(plan)


        #move.go_to_near_back_state()
        # input("plan ok enter enter")
        # move.go_to_default()
        # input("plan ok enter enter")
        # grasp_client(width=0.04)
        # move.go_to_near_x_state()
        # cartesian_move = [0,-0.1,0]

        # cartesian_plan, fraction = move.plan_cartesian_path(cartesian_move)
        # move.display_trajectory(cartesian_plan)
        # move.execute_plan(cartesian_plan)
        # input("plan ok enter enter")
        # move.go_to_default()
        # input("plan ok enter enter")
        # grasp_client(width=0.08)
        #move.go_to_near_back_state()

        # orientation = get_quaternion_from_euler(0,pi/2,-pi/2)
        # position = [0.3, -0.1, 0.3]
        # move.go_to_pose_goal(orientation, position)
        # input("plan ok enter enter")

        print("============ go state ============")
        # orientation = get_quaternion_from_euler(pi/2,pi/4,pi)
        # position = [0.25,0, 0.3]
        # move.go_to_pose_goal(orientation, position)

        # print("============ Configure Completed ============")

        # print("============ Default Position Move ============")
        # move.go_to_joint_state()
        # input("============ Default Position Completed ============")

        # print("============ Startiing Position Move ============")
        # orientation = get_quaternion_from_euler(pi/2,pi/4,pi)
        # print("target orientation is", orientation)
        # position = [0.25,-0.1, 0.3]
        # move.go_to_pose_goal(orientation, position)
        # input("============ Startiing Position Competed ============")
        # # print("============ Startiing Position Move ============")
        # # move.go_to_near_x_state
        # # input("============ Startiing Position Competed ============")

        # print("============ Default Position Move ============")
        # move.go_to_joint_state()
        # input("============ Default Position Completed ============")

        # print("============ Startiing Position Move ============")
        # move.go_to_near_y_state()
        # input("============ Startiing Position Competed ============")

        # # print("============ Default Position Move ============")
        # # move.go_to_joint_state()
        # # input("============ Default Position Completed ============")

        # print("============ Startiing Position Move ============")
        # move.go_to_near_x_state()
        # input("============ Startiing Position Competed ============")
        # print("============ Cartesian Move ============")
        # cartesian_move = [0.2,0,0]
        # cartesian_plan, fraction = move.plan_cartesian_path(cartesian_move)
        # move.display_trajectory(cartesian_plan)
        # move.execute_plan(cartesian_plan)
        # # input("============ Cartesian Move Completed ============")


        # # input("============ Press `Enter` to add a box to the planning scene ...")
        # # tutorial.add_box()

        # # input("============ Press `Enter` to attach a Box to the Panda robot ...")
        # # tutorial.attach_box()

        # # input(
        # #     "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # # )
        # # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # # tutorial.execute_pl    orientation = [0, 0, 1, 0]
        # # position = [0.3, 0.4, 0.4]an(cartesian_plan)

        # # input("============ Press `Enter` to detach the box from the Panda robot ...")
        # # tutorial.detach_box()

        # # input(
        # #     "============ Press `Enter` to remove the box from the planning scene ..."
        # # )
        # # tutorial.remove_box()
        # # print("============ Python move demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    os.system("python3 "+os.path.dirname(__file__)+"/jenga_obstacle_environment.py")
    main()
