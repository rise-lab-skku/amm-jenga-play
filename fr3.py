#modified 06-20

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
            links,name=name,gripper_links=links[17],urdf_string=urdf_string,urdf_filepath=urdf_filepath
        )
        
        #calling Robot class, not sure what it exactly do

        self.grippers[0].tool=SE3(0,0,0.1034)
        #seems to be edited if gripper changes

        self.qdlim=np.array([2,1,1.5,1.25,3,1.5,3])
        self.qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4])
        self.qz = np.zeros(7)

        self.addconfiguration("qdlim(means velocity limit)",self.qdlim)
        self.addconfiguration("qr(ready position)", self.qr)
        self.addconfiguration("qz(zero joint angle)", self.qz)


if __name__=="__main__": #this is for debugging, add value desired to see, run only when directly run this file
    r=fr3()
    print(len(r.links))
    for link in r.grippers[0].links:
        print(link)
    '''
    for link in r.links:
        print(link)
    '''
    
    print(r.grippers[0].tool)