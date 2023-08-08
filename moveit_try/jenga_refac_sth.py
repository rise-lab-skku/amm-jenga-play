import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory as DT
import spatialmath as sm

from gripper_commander import GripperCommander

from math import pi, dist, cos, atan, sqrt
from copy import deepcopy as dcp
from block_recog_pkg.srv import (
    GetWorldCoord,
    GetWorldCoordRequest,
    CaptureImage,
    CaptureImageRequest,
)


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


class ManipulatorCommander:
    def __init__(self, eef, move_group):
        self.eef = eef
        self.move_group = move_group

    def plan_and_execute(self, pose_goal):
        is_success, traj, planning_time, error_code = move_group.plan(pose_goal)
        _msg = "success" if is_success else "failed"
        rospy.loginfo(
            f"Planning is [ {_msg} ] (error code : {error_code.val}, planning time : {planning_time:.2f}s)"
        )
        self.display_trajectory(traj)

        if is_success:
            rospy.loginfo("Executing the plan")
            move_group.execute(traj, wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

    def plan_and_execute_cartesian(self, cartesian_move, avoid_collisions=False):
        move_group = move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x += cartesian_move[0]
        wpose.position.y += cartesian_move[1]
        wpose.position.z += cartesian_move[2]
        waypoints.append(dcp(wpose))

        plan, fraction = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0, avoid_collisions
        )

        self.display_trajectory(plan)
        print("fraction: ", fraction)
        if fraction > 0.98:
            if input("Enter to execute or press q to abort") == "q":
                print("quit")
                is_success = False

            else:
                self.execute_plan(plan)
                is_success = True
        else:
            print("fraction is too low")
            is_success = False

        move_group.clear_pose_targets()
        return is_success

    def jenga_extract(self, target_point, temp_point, method):
        dx = temp_point.x - target_point.x
        dy = temp_point.y - target_point.y
        direction = atan(dy / dx) + pi / 2
        print(direction)
        rpy = [pi / 2, pi / 2, direction]

        r = sqrt(x**2 + y**2)
        x /= r
        y /= r

        if method == "grasp":  # grasp
            self.rpy_goal(rpy=rpy, xyz=temp_point)
            print("arrived temp_point")
            rospy.sleep(1)
            extract = [-0.095 * x, -0.095 * y, 0]
            if self.plan_traj_exec_cartesian_path(
                cartesian_move=extract, avoid_collisions=False
            ):
                rospy.sleep(1)
                gripper.grasp(
                    0.07,
                )
                rospy.sleep(1)
                extract = [0.13 * x, 0.13 * y, 0]
                self.plan_traj_exec_cartesian_path(
                    cartesian_move=extract, avoid_collisions=False
                )
            rospy.sleep(1)
            move_client(0.08)
            print("grasp extraction complete")

        elif method == "push":  # "push"
            self.rpy_goal(rpy=rpy, xyz=temp_point)
            print("rpy", rpy)
            gripper.move(0, 0.1)
            rospy.sleep(1)
            extract = [-0.145 * x, -0.145 * y, 0]
            if self.plan_traj_exec_cartesian_path(
                cartesian_move=extract, avoid_collisions=False
            ):
                rospy.sleep(1)
                extract = [0.12 * x, 0.12 * y, 0]
                self.plan_traj_exec_cartesian_path(
                    cartesian_move=extract, avoid_collisions=False
                )
            rospy.sleep(1)
            gripper.homing()
            print("push extraction complete")
        else:
            print("wrong method")


class MoveitScene:
    def __init__(self, robot):
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        # robot = moveit_commander.RobotCommander()
        self.robot=robot

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
        # group_name = "panda_arm" to make TCP as Panda_manipulaotor new fingert tip
        self.group_name = "panda_manipulator"
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
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
        # print(robot.get_current_state())
        print("")
        # END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def add_jenga_box(self, points=None, timeout=4):
        box_name = self.box_name
        scene = self.scene
        # Add jenga box to the scene.
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        if points is None:
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.x = 0.4
            box_pose.pose.position.y = -0.4
            box_pose.pose.position.z = 0.31
        else:
            box_pose.pose.position.x = (
                points[0][0] + points[1][0] + points[2][0] + points[3][0]
            ) / 4
            box_pose.pose.position.y = (
                points[0][1] + points[1][1] + points[2][1] + points[3][1]
            ) / 4
            box_pose.pose.position.z = 0.31
            rpy = self.two_points_to_rpy(points[0], points[1])
            print(rpy)
            w, x, y, z = sm.UnitQuaternion.RPY(0, 0, rpy[2])
            box_pose.pose.orientation.x = x
            box_pose.pose.orientation.y = y
            box_pose.pose.orientation.z = z
            box_pose.pose.orientation.w = w
        box_name = "jenga_box"
        scene.add_box(box_name, box_pose, size=(0.080, 0.080, 0.62))
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
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


def initialize():
    # scene
    robot = moveit_commander.RobotCommander()
    scene = MoveitScene(robot)
    # move_group
    move_group = moveit_commander.MoveGroupCommander("arm")
    mg = ManipulatorCommander(move_group)
    return scene, mg


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("robot_commander_analysis", anonymous=True)

    robot = moveit_commander.RobotCommander()
    print(f"============ Root Link:  {robot.get_root_link():^20} ============")
    print(f"============ Move Groups:{str(robot.get_group_names()):^20} ============")
    scene = moveit_commander.PlanningSceneInterface()
    scene.remove_world_object()
    move_group = moveit_commander.MoveGroupCommander("arm")
    gripper = GripperCommander(wait=False)

    scene, arm = initialize()

    links = robot.get_link_names()
    grasp_link = links[list(map(lambda x: x.find("grasp") > -1, links)).index(True)]
    push_link = links[list(map(lambda x: x.find("push") > -1, links)).index(True)]
    grasper = ManipulatorCommander(grasp_link, move_group)
    pusher = ManipulatorCommander(push_link, move_group)


"""
while True:
    if c == "A"
        fooo
    elif c == "B":
        barrrr

=============

def do_A():
    pass

def do_B():
    pass

cmd_dictionary = {
    "A": do_A,
    "B": do_B,
}

eval("A()")

while True:
    cmd_diictionary[c]()



"""

    while True:
        command = input("\ncommand:")

        # Basic functions
        if command == "init":
            move_group.set_named_target("ready")
            move_group.plan()
            gripper.homing()

        elif command == "quit":
            break

        elif command == "eef":
            eef = input("full eef link name:")
            move_group.set_end_effector_link(eef)

        elif command == "plan":
            ans = list(map(float, input("xyzrpy coordinate(space separated):").split()))
            goal = moveit_commander.conversions.list_to_pose(ans)
            plan = move_group.plan(goal)

        elif command == "display":
            display_trajectory(plan, robot)

        elif command == "execute":
            move_group.execute(plan, wait=True)

        elif command == "cartesian":
            ans = input("cartesian move:")
            cartesian_move = [float(ans.split(" ")[i]) for i in range(3)]
            move_group.compute_cartesian_path(cartesian_move)

        # gripper functions
        elif command == "grasp":
            width = float(input("grasp width:"))
            speed = float(input("grasp speed:"))
            force = float(input("grasp force:"))
            print(
                f"Trying grasping into {width} with tolerance {gripper.epsilon} by {speed} and {force}..."
            )
            print(gripper.grasp(width, speed, force))

        elif command == "move":
            width = float(input("move width:"))
            speed = float(input("move speed:"))
            print(f"Trying moving to {width} by {speed}...")
            print(gripper.move(width, speed))

        elif command == "final_test":
            # #camera position
            grasper.rpy_goal([pi / 2, pi / 2, pi / 2], [-0.05, -0.5, 0.5])
            rospy.sleep(5)
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
            rospy.wait_for_service("GetWorldCoordinates")

            get_coord = rospy.ServiceProxy("GetWorldCoordinates", GetWorldCoord)

            request = GetWorldCoordRequest()

            request.target_block = "init 1"

            response = get_coord(request.target_block)

            jenga_coordinate1_x = dcp(dcp(response.center_x))
            jenga_coordinate1_y = dcp(dcp(response.center_y))
            jenga_coordinate1_z = dcp(dcp(response.center_z))
            jenga_coordinate2_x = dcp(response.target_x)
            jenga_coordinate2_y = dcp(response.target_y)
            jenga_coordinate2_z = dcp(response.target_z)
            # print(dcp(response.center_y))
            # print(dcp(response.center_z))
            # print(dcp(response.target_x))
            # print(dcp(response.target_y))
            # print(dcp(response.target_z))

            # INIT 2
            rospy.wait_for_service("GetWorldCoordinates")

            get_coord = rospy.ServiceProxy("GetWorldCoordinates", GetWorldCoord)

            request = GetWorldCoordRequest()

            request.target_block = "init 2"

            response = get_coord(request.target_block)

            jenga_coordinate3_x = dcp(response.center_x)
            jenga_coordinate3_y = dcp(response.center_y)
            jenga_coordinate3_z = dcp(response.center_z)
            jenga_coordinate4_x = dcp(response.target_x)
            jenga_coordinate4_y = dcp(response.target_y)
            jenga_coordinate4_z = dcp(response.target_z)

            # rospy.loginfo(jenga_coordinate1_x)
            # rospy.loginfo(jenga_coordinate1_y)
            # rospy.loginfo(jenga_coordinate1_z)

            # marker로 좌표 check
            # marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
            # marker = Marker()

            ###############give me 4 poinst to make jenga################
            points = [
                [jenga_coordinate1_x, jenga_coordinate1_y, jenga_coordinate1_z],
                [jenga_coordinate2_x, jenga_coordinate2_y, jenga_coordinate2_z],
                [jenga_coordinate3_x, jenga_coordinate3_y, jenga_coordinate3_z],
                [jenga_coordinate4_x, jenga_coordinate4_y, jenga_coordinate4_z],
            ]
            # give me succeeding points

            scene.add_jenga_box(points)

            while True:
                command = input("\njenga to extract: ")  # ex. "green 5"
                if command == "q":
                    break

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

                # dcp(response.success)
                # (dcp(response.center_x))
                # (dcp(response.center_y))
                # (dcp(response.center_z))
                # (dcp(response.target_x))
                # (dcp(response.target_y))
                # (dcp(response.target_z))
                # dcp(response.push)

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
                method = response.push  # true-push fasle-pull
                ###############################################################
                if method:
                    pusher.jenga_extract(target_point, temp_point, method)
                else:
                    grasper.jenga_extract(target_point, temp_point, method)

            rospy.sleep(1)
            # move.move_client(0.08)
            grasper.go_to_default()

        else:
            print("command error")


if __name__ == "__main__":
    main()
