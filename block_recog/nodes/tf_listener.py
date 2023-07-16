#!/usr/bin/env python  
import roslib
import rospy
import tf
import geometry_msgs.msg
import numpy as np


def transform_mat_from_trans_rot(trans, rot):
    e1, e2, e3, e4 = rot
    trans_matrix = np.array([[1-2*(e2**2)-2*(e3**2), 2*(e1*e2-e3*e4), 2*(e1*e3+e2*e4), trans[0]],
                                [2*(e1*e2+e3*e4), 1-2*(e1**2)-2*(e3**2), 2*(e2*e3-e1*e4), trans[1]],
                                [2*(e1*e3-e2*e4), 2*(e2*e3+e1*e4), 1-2*(e1**2)-2*(e2**2), trans[2]],
                                [0, 0, 0, 1]])
    
    return trans_matrix


if __name__ == '__main__':
    rospy.init_node('block_recognition_node')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # lookupTransform('target', 'source', Time)
            (trans_cali,rot_cali) = listener.lookupTransform('camera_base', 'panda_hand', rospy.Time(0))
            (trans_world,rot_world) = listener.lookupTransform('world', 'panda_hand', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # rospy.loginfo((trans, rot))
        
        cali_transform_matrix = transform_mat_from_trans_rot(trans_cali, rot_cali)
        to_world_transform_matrix = transform_mat_from_trans_rot(trans_world, rot_cali)
        
        
        
        rate.sleep()
        
        # here return can i use the matrix as function works?