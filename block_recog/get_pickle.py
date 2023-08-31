#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pickle

bridge = CvBridge()


def image_callback1(msg):
    # Convert ROS images to OpenCV format
    img_color = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Save pickle
    with open("/home/hr/Desktop/cali.p", "wb") as rgb:
        pickle.dump(img_color, rgb)


def image_callback2(msg):
    # Convert ROS images to OpenCV format
    img_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    # Save pickle
    with open("dep.p", "wb") as dep:
        pickle.dump(img_depth, dep)


def main():
    rospy.init_node("image_processing_node",anonymous=True)
    rospy.Subscriber("/rgb/image_raw", Image, image_callback1)
    # rospy.Subscriber("/depth_to_rgb/image_raw", Image, image_callback2)
    rospy.spin()


if __name__ == "__main__":
    main()
