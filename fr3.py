# modified 06-26

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from spatialmath import SE3
import os
import re


class fr3(Robot):
    def __init__(self):

        links,name,urdf_string,urdf_filepath = self.URDF_read(
            "franka_description/robots/fr3/fr3.urdf.xacro",
            tld=os.path.join(os.path.dirname(__file__),"franka_ros-develop"),
            xacro_tld="franka_description"
        )
        # added top level directory
        #print(links[17])
        qdlim=[]
        l=[i.start() for i in re.finditer(" velocity",urdf_string)]
        for i in range(7):
            idx=urdf_string.find('"',l[i]+11)
            qdlim.append(urdf_string[l[i]+11:idx])
        #read qdlim from urdf, not by hardcoding

        super().__init__(
            links,gripper_links=links[17],tool=SE3(0,0,0.1974)
        )
        self.grippers[0].tool=SE3(0,0,0.1034)
        #end-effector's position

        self.qdlim=np.array(qdlim)
        self.qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4])
        self.qz = np.zeros(7)

        self.addconfiguration("qr(ready position)", self.qr)
        self.addconfiguration("qz(zero joint angle)", self.qz)


if __name__=="__main__": #this is for debugging, add value desired to see, run only when directly run this file
    r=fr3()
    print(r.grippers[0])
    print(r.links[17])
    #print(r)
    # print(r.tool)
    # print(len(r.links))
    '''
    for link in r.grippers[0].links:
        print(link)'''
    '''
    for link in r.links:
        print(link)
    '''

    #print(r.grippers[0].tool)
    #print(r.qdlim)