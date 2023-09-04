from franka_gripper.msg import *
from actionlib import SimpleActionClient as SAC


class Commander:
    def __init__(self, eps=0.05, fake=False):
        self.epsilon = GraspEpsilon(eps, eps)
        self.recent_client = None

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
        self.recent_client = self.grasp_client
        return self._pend(goal)

    def move(self, width, speed):
        goal = MoveGoal(width, speed)
        self.recent_client = self.move_client
        return self._pend(goal)

    def homing(self):
        goal = HomingGoal()
        self.recent_client = self.homing_client
        return self._pend(goal)

    def stop(self):
        goal = StopGoal()
        self.stop_client.send_goal(goal)
        if self.recent_client:
            self.recent_client.cancel_goal()
            self.recent_client = None
        self.stop_client.wait_for_result()
        result=self.stop_client.get_result()
        if result.stopped:
            rospy.logwarn("Gripper emergency stopped.")
        else:
            rospy.logerr("Gripper emergency stop failed.")
        return result

    def _pend(self, goal):
        self.recent_client.send_goal(goal)
        while not self.recent_client.get_state():
            pass
        return self.recent_client.get_state()

    def wait(self):
        self.recent_client.wait_for_result()
        return self.recent_client.get_result()

def test(gripper):
    try:
        while True:
            command = input("\ncommand:")
            if command == "grasp":
                width = float(input("grasp width:"))
                speed = float(input("grasp speed:"))
                force = float(input("grasp force:"))

                if gripper.grasp(width, speed, force) == 1:
                    print(
                        f"Trying grasping into ({width} \u00B1 {gripper.epsilon.inner}) [m] by {speed} [m/s] until reaching {force} [N]..."
                    )
                else:
                    print("rejection")

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
                continue

            print(gripper.wait())

    except KeyboardInterrupt:
        print(gripper.stop())
        test(gripper)


if __name__ == "__main__":
    import rospy

    rospy.init_node("gripper_commander_test", disable_signals=True)
    gripper = Commander(eps=float(input("grasp epsilon:")))
    test(gripper)
