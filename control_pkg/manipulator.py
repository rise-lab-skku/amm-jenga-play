import rospy
from math import pi, dist, cos, atan

class Commander:
    def __init__(self, eef, move_group, gripper):
        self.eef = eef
        self.move_group = move_group
        self.gripper = gripper

    def plan_and_execute(self, pose_goal):
        is_success, traj, planning_time, error_code = self.move_group.plan(pose_goal)
        _msg = "success" if is_success else "failed"
        rospy.loginfo(
            f"Planning is [ {_msg} ] (error code : {error_code.val}, planning time : {planning_time:.2f}s)"
        )
        self.display_trajectory(traj)

        if is_success:
            rospy.loginfo("Executing the plan")
            self.move_group.execute(traj, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def plan_and_execute_cartesian(self, cartesian_move, avoid_collisions=False):
        move_group = move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x += cartesian_move[0]
        wpose.position.y += cartesian_move[1]
        wpose.position.z += cartesian_move[2]
        waypoints.append(wpose)

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

    def extract_jenga(self, target_point, temp_point, method):
        dx = temp_point.x - target_point.x
        dy = temp_point.y - target_point.y
        direction = atan(dy / dx) + pi / 2
        print(direction)
        rpy = [pi / 2, pi / 2, direction]

        r = dist(x**2 + y**2)
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
                self.gripper.grasp(
                    0.07,
                )
                rospy.sleep(1)
                extract = [0.13 * x, 0.13 * y, 0]
                self.plan_traj_exec_cartesian_path(
                    cartesian_move=extract, avoid_collisions=False
                )
            rospy.sleep(1)
            
            print("grasp extraction complete")

        elif method == "push":  # "push"
            self.rpy_goal(rpy=rpy, xyz=temp_point)
            print("rpy", rpy)
            self.gripper.move(0, 0.1)
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
            self.gripper.homing()
            print("push extraction complete")
        else:
            print("wrong method")
def test(manipulator, gripper):
    pass

if __name__=="__main__":
    pass