#!/usr/bin/python3

import rospy
from block_recog_pkg.srv import (
    GetWorldCoord,
    GetWorldCoordResponse,
    GetWorldCoordRequest,
    CaptureImage,
    CaptureImageResponse,
    CaptureImageRequest,
)
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

import numpy as np
import time
import func
import pickle
import cv2
import os
from tf_conversions import *



class CoordinateServer:
    """A class that provides services to capture RGB and depth images and obtain target block's world coordinates.

    Attributes:
        tower_transform_initialized (bool): Flag indicating whether the tower's transformation is initialized.
        img_depth (numpy.ndarray): The depth image captured from the camera.
        img_color (numpy.ndarray): The RGB image captured from the camera.
        ready_to_capture_image (bool): Flag indicating whether the server is ready to capture images.
        capture_once (int): Counter to ensure single image capture.

    Subscribers:
        /rgb/image_raw (sensor_msgs.Image): Subscriber for RGB images from the camera.
        /depth_to_rgb/image_raw (sensor_msgs.Image): Subscriber for depth images from the camera.

    Services:
        CaptureImage (block_recog.srv.CaptureImage): Service to capture RGB and depth images.
        GetWorldCoordinates (block_recog.srv.GetWorldCoord): Service to obtain world coordinates for a target block.
    """


    def GetWorldCoordinates(
        self, request: GetWorldCoordRequest
    ) -> GetWorldCoordResponse:
        pass


if __name__ == "__main__":
    rospy.init_node("coordinate_server_node", anonymous=True)  # init node
    CoordinateServer(fake=True)
    rospy.spin()