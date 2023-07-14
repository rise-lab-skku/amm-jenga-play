# modified 07-14

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from spatialmath import SE3
import os
import re


class fr3(Robot):
    def get_tcp(self, link):
        idx=[0,0,0,0]
        idx[0] = self.urdf.find(link)
        idx[1] = self.urdf.find("xyz", idx[0])
        idx[2] = self.urdf.find('"/', idx[1])
        xyz=[float(self.urdf[idx[1] + 5: idx[2]].split(' ')[i]) for i in range(3)]
        return SE3.Trans(xyz)

    def __init__(self):
        links, name, self.urdf, _ = self.URDF_read(
            "franka_description/robots/fr3/fr3.urdf.xacro",
            tld=os.path.join(os.path.dirname(__file__), "franka_ros-develop"),
            xacro_tld="franka_description"
        )

        qdlim = []
        idx = [i.start() for i in re.finditer(" velocity", self.urdf)]
        for i in range(7):
            qdlim.append(self.urdf[idx[i] + 11: self.urdf.find('"', idx[i] + 11)])
        self.qdlim = np.array(qdlim)

        super().__init__(
            links, name=name, gripper_links=links[17], tool=self.get_tcp("hand_tcp2_joint")
        )
        self.grippers[0].tool = self.get_tcp("hand_tcp_joint")

        self.qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4])


if __name__ == "__main__":
    robot = fr3()
    print(robot.tool)
    print(robot.grippers[0].tool)
