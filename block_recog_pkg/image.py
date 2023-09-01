import rospy
from .utils import *
import cv2
import pickle
from sensor_msgs.msg import Image
from os.path import join
import numpy as np


class Commander:
    def __init__(self, fake=False):
        self.rgb = None
        self.d = None

        self.ready = False

        if fake:
            self.rgb = pickle.load(open(join(pkg_path, RGB_PKL), "rb"))
            self.d = pickle.load(open(join(pkg_path, DEP_PKL), "rb"))
        else:
            rospy.Subscriber(RGB_NODE, Image, self.rgb_image_callback)
            rospy.Subscriber(DEP_NODE, Image, self.depth_image_callback)

    def rgb_image_callback(self, msg) -> None:
        if self.rgb is None and self.ready:
            self.rgb = cv2.rotate(
                bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"), cv2.ROTATE_180
            )
            rospy.logwarn(f"RGB image captured (shape: {self.rgb.shape}))")

    def depth_image_callback(self, msg) -> None:
        if self.d is None and self.ready:
            self.d = cv2.rotate(
                bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1"), cv2.ROTATE_180
            )
            rospy.logwarn(f"Depth image captured (shape: {self.d.shape}))")

    def capture(self):
        t0 = rospy.Time.now()
        self.ready = True
        rospy.loginfo("waiting")
        while self.rgb is None or self.d is None:
            if rospy.Time.now() - t0 > TIME_LIMIT:
                rospy.logwarn("image capture failed")
                raise
        rospy.loginfo("captured image")
        self.bgr = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR)
        self.hsv = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2HSV)

    def get_masks(self, id):
        # return a list of masks, the last one is the combination of all masks
        color = colors[id]
        mask = cv2.inRange(self.hsv, np.array(color["lower"]), np.array(color["upper"]))

        mask = cv2.dilate(cv2.erode(mask, kern), kern)

        cnt, labels = cv2.connectedComponents(mask)

        masks = []
        for i in range(1, cnt):
            masks.append(((labels == i)*255).astype('uint8'))

        masks.append(mask)
        return masks

    def get_pcds(self, masks):
        pcds=[]
        for mask in masks:
            rgb_image=o3d.geometry.Image(cv2.bitwise_and(self.rgb,self.rgb,None,mask))
            d_image=o3d.geometry.Image(cv2.bitwise_and(self.d,self.d,None,mask))
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_image, d_image, convert_rgb_to_intensity=False)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
            cl,ind=pcd.remove_radius_outlier(256, 0.025)
            pcds.append(cl.select_by_index(ind))

        return pcds


if __name__=="__main__":
    rospy.init_node('imageprocess',anonymous=True,disable_signals=True)
    image=Commander(fake=True)
    image.capture()
    yellow_masks=image.get_masks(1)