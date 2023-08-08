from franka_gripper.msg import *
from actionlib import SimpleActionClient as SAC


class GripperCommander:
    def __init__(self, eps=0.05, wait=True):
        self.epsilon = GraspEpsilon(eps, eps)

        self.grasp_client = SAC("/franka_gripper/grasp", GraspAction)
        self.move_client = SAC("/franka_gripper/move", MoveAction)
        self.homing_client = SAC("/franka_gripper/homing", HomingAction)
        self.stop_client = SAC("/franka_gripper/stop", StopAction)

        if wait:
            self.grasp_client.wait_for_server()
            self.move_client.wait_for_server()
            self.homing_client.wait_for_server()
            self.stop_client.wait_for_server()

    def grasp(self, width, speed, force):
        goal = GraspGoal(width, self.epsilon, speed, force)
        self.grasp_client.send_goal_and_wait(goal)
        return self.grasp_client.get_result()

    def move(self, width, speed):
        goal = MoveGoal(width, speed)
        self.move_client.send_goal_and_wait(goal)
        return self.move_client.get_result()

    def homing(self):
        goal = HomingGoal()
        self.homing_client.send_goal_and_wait(goal)
        return self.homing_client.get_result()

    def stop(self):
        goal = StopGoal()
        self.stop_client.send_goal(goal)
        return self.stop_client.get_state()


if __name__ == "__main__":
    import rospy

    rospy.init_node("gripper_commander_test")
    gripper = GripperCommander()
