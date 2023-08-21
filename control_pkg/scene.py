from moveit_commander.planning_scene_interface import PlanningSceneInterface as PSI
from moveit_commander.conversions import *

import moveit_msgs.msg
import rospy
from numpy import mean, arctan, pi


JENGA_NAME = "jenga_tower"
JENGA_SIZE = (0.080, 0.080, 0.62)
JENGA_Z = 0.31


class Commander(PSI):
    def __init__(self):
        super.__init__()
        self.remove_world_object()

    def add_jenga(self, points, timeout=4):
        # Add jenga box to the scene.

        if False:
            pass
        # if points is None:
        # box_pose.pose.orientation.w = 1.0
        # box_pose.pose.position.x = 0.4
        # box_pose.pose.position.y = -0.4
        # box_pose.pose.position.z = 0.31
        else:
            x_mean = mean(points[i].x for i in range(4))
            y_mean = mean(points[i].y for i in range(4))

            dx = points[0].x - points[1].x
            dy = points[0].y - points[1].y
            yaw = arctan(dy / dx) + pi / 2

            box_pose = list_to_pose([x_mean, y_mean, JENGA_Z, pi / 2, pi / 2, yaw])
        self.add_box(JENGA_NAME, box_pose, size=JENGA_SIZE)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.get_attached_objects([JENGA_NAME])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = JENGA_NAME in self.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
