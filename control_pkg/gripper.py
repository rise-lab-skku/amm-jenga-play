from franka_gripper.msg import *
from actionlib import SimpleActionClient as SAC


class Commander:
    def __init__(self, eps=0.05, fake=False):
        self.epsilon = GraspEpsilon(eps, eps)

        self.grasp_client = SAC("/franka_gripper/grasp", GraspAction)
        self.move_client = SAC("/franka_gripper/move", MoveAction)
        self.homing_client = SAC("/franka_gripper/homing", HomingAction)
        self.stop_client = SAC("/franka_gripper/stop", StopAction)

        if not fake:
            self.grasp_client.wait_for_server()
            self.move_client.wait_for_server()
            self.homing_client.wait_for_server()
            self.stop_client.wait_for_server()

    def grasp(self, width, speed, force):
        goal = GraspGoal(width, self.epsilon, speed, force)
        self.grasp_client.send_goal(goal)
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

        # self.grasp_client.cancel_goal()
        # self.move_client.cancel_goal()
        # self.homing_client.cancel_goal()

        return self.stop_client.get_state()


def test(gripper):
    try:
        while True:
            command = input("\ncommand:")
            if command == "grasp":
                width = float(input("grasp width:"))
                speed = float(input("grasp speed:"))
                force = float(input("grasp force:"))
                print(
                    f"Trying grasping into ({width} \u00B1 {gripper.epsilon}) [m] by {speed} [m/s] until reaching {force} [N]..."
                )
                print(gripper.grasp(width, speed, force))

            elif command == "move":
                width = float(input("move width:"))
                speed = float(input("move speed:"))
                print(f"Trying moving to {width} [m] by {speed} [m/s]...")
                print(gripper.move(width, speed))
            elif command == "homing":
                print("Trying homming...")
                gripper.homing()
            elif command == "quit":
                break
            else:
                print("command error.")
    except KeyboardInterrupt:
        gripper.stop()
        print("stopped.")
        test(gripper)


if __name__ == "__test__":
    import rospy

    rospy.init_node("gripper_commander_test", disable_signals=True)
    gripper = Commander(eps=float(input("grasp epsilon:")),fake=True)
    test(gripper)
