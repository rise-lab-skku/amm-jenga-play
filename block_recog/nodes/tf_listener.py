#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import tf
import geometry_msgs.msg
import numpy as np

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    
    # trans_rot = rospy.Publisher('/world2camera', geometry_msgs.msg.Transform,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('camera_base', 'world', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # trans_rot.publish(trans, rot)
        rospy.loginfo((trans, rot))
        e1, e2, e3, e4 = rot
        trans_matrix = np.array([[1-2*(e2**2)-2*(e3**2), 2*(e1*e2-e3*e4), 2*(e1*e3+e2*e4), trans[0]],
                                 [2*(e1*e2+e3*e4), 1-2*(e1**2)-2*(e3**2), 2*(e2*e3-e1*e4), trans[1]],
                                 [2*(e1*e3-e2*e4), 2*(e2*e3+e1*e4), 1-2*(e1**2)-2*(e2**2), trans[2]],
                                 [0, 0, 0, 1]])
        rospy.loginfo(trans_matrix)
        # print(trans, rot)
        
        rate.sleep()
        
        # here return can i use the matrix as function works?