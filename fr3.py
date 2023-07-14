# modified 07-14

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from spatialmath import SE3
import os
import re


class fr3(Robot):
    def __init__(self):
        links, name, urdf, _ = self.URDF_read(
            "franka_description/robots/fr3/fr3.urdf.xacro",
            tld=os.path.join(os.path.dirname(__file__), "franka_ros-develop"),
            xacro_tld="franka_description"
        )

        qdlim = []
        idx = [i.start() for i in re.finditer(" velocity", urdf)]
        for i in range(7):
            idx = urdf.find('"', idx[i] + 11)
            qdlim.append(urdf[idx[i] + 11: idx])
        self.qdlim = np.array(qdlim)

        idx[0] = urdf.find("hand_tcp_joint")
        idx[1] = urdf.find("xyz", idx[0])
        idx[2] = urdf.find('"/', idx[1])
        tcp_xyz = [float(i) for i in range(3)]

        super().__init__(
            links, gripper_links=links[17], tool=
        )


        self.qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4])


if __name__ == "__main__":
    robot = fr3()
    print(robot)