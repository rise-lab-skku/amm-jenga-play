import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
class MoveitScene:
    def __init__(self, robot):
       
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
        box_pose = geometry_msgs.msg.Pose()
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
            moveit_commander.conversions.list_to_pose()
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
