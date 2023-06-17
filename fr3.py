#modified 06-17

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from spatialmath import SE3
import os

class fr3(Robot):
    def __init__(self):

        links,name,urdf_string,urdf_filepath=self.URDF_read(
            "franka_description/robots/fr3/fr3.urdf.xacro",
            tld=os.path.join(os.path.dirname(__file__),"franka_ros-develop"),
            xacro_tld="franka_description"
        )
        #added top level directory

        super().__init__(
            links,name=name,manufacturer="Franka Emika",gripper_links=links[9],urdf_string=urdf_string,urdf_filepath=urdf_filepath
        )
        #calling Robot class, not sure what it exactly do

        self.grippers[0].tool=SE3(0,0,0.1034)
        #seems to be edited if gripper changes

        self.qdlim=np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 3.0, 3.0])
        self.qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4])
        self.qz = np.zeros(7)

        self.addconfiguration("qdlim(what's mean??)",self.qdlim)
        self.addconfiguration("qr(means ready position)", self.qr)
        self.addconfiguration("qz(means zero joint angle)", self.qz)


if __name__=="__main__": #this is for debugging, add value desired to see, run only when directly run this file
    r=fr3()
    for link in r.grippers[0].links:
        print(link)