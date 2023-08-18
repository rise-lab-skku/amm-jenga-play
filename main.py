import rospy
import moveit_commander


import control_pkg
from moveit_msgs.msg import DisplayTrajectory as DT
from math import pi, dist, cos
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


def initialize():
    moveit_commander.roscpp_initialize("")
    rospy.init_node("jenga_main", anonymous=True)

    robot = moveit_commander.RobotCommander()
    print(f"============ Root Link:  {robot.get_root_link():^20} ============")
    print(f"============ Move Groups:{str(robot.get_group_names()):^20} ============")

    # move_group
    move_group = moveit_commander.MoveGroupCommander("arm")
    gripper = control_pkg.gripper.Commander(wait=False)
    return robot, move_group, gripper


robot, move_group, gripper = initialize()

scene = moveit_commander.PlanningSceneInterface()
scene.remove_world_object()

links = robot.get_link_names()
grasp_link = links[list(map(lambda x: x.find("grasp") > -1, links)).index(True)]
push_link = links[list(map(lambda x: x.find("push") > -1, links)).index(True)]
grasper = control_pkg.manipulator.Commander(grasp_link, move_group)
pusher = control_pkg.manipulator.Commander(push_link, move_group)

while True:
    command = input("\ncommand:")

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