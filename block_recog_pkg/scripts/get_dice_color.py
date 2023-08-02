#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from block_recog_pkg.srv import GetDiceColor, GetDiceColorResponse


def detect_dice_color(img_dice: np.ndarray) -> str:
    """
    Detects the color of the top of the dice in the given RGB image.

    Args:
        img_dice (numpy.ndarray): The RGB image of the dice.

    Returns:
        str: The name of the color ('green', 'pink', 'yellow', 'blue', 'violet', or 'red').
    """
    img_hsv = cv2.cvtColor(img_dice, cv2.COLOR_BGR2HSV)

    colors = ["green", "pink", "yellow", "blue", "violet", "red"]

    # RED
    lower_red1 = np.array([0, 130, 50])
    upper_red1 = np.array([15, 255, 255])
    lower_red2 = np.array([160, 130, 50])
    upper_red2 = np.array([179, 255, 255])
    # PINK
    lower_pink1 = np.array([0, 55, 80])
    upper_pink1 = np.array([10, 130, 255])
    lower_pink2 = np.array([150, 55, 80])
    upper_pink2 = np.array([179, 130, 255])
    # GREEN
    lower_green = np.array([70 - 20, 50, 50])
    upper_green = np.array([70 + 15, 255, 255])
    # YELLOW
    lower_yellow = np.array([30 - 10, 80, 80])
    upper_yellow = np.array([30 + 10, 255, 255])
    # BLUE
    lower_blue = np.array([100 - 10, 100, 100])
    upper_blue = np.array([100 + 9, 255, 255])
    # VIOLET
    lower_violet = np.array([130 - 20, 60, 60])
    upper_violet = np.array([130 + 20, 255, 255])

    domintant_color_area = -1
    dominant_color = None

    for color in colors:
        if color == "pink" or color == "red":
            if color == "pink":
                lower_color1 = lower_pink1
                lower_color2 = lower_pink2
                upper_color1 = upper_pink1
                upper_color2 = upper_pink2
            if color == "red":
                lower_color1 = lower_red1
                lower_color2 = lower_red2
                upper_color1 = upper_red1
                upper_color2 = upper_red2

            mask_color1 = cv2.inRange(img_hsv, lower_color1, upper_color1)
            mask_color2 = cv2.inRange(img_hsv, lower_color2, upper_color2)
            img_mask_color = mask_color1 + mask_color2

        else:
            if color == "blue":
                lower_color = lower_blue
                upper_color = upper_blue
            if color == "green":
                lower_color = lower_green
                upper_color = upper_green
            if color == "violet":
                lower_color = lower_violet
                upper_color = upper_violet
            if color == "yellow":
                lower_color = lower_yellow
                upper_color = upper_yellow

        img_mask_color = cv2.inRange(img_hsv, lower_color, upper_color)

        # Find contours of each color's mask
        contours_color, _ = cv2.findContours(img_mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Determine the area of each color's contour
        area_color = sum([cv2.contourArea(cnt) for cnt in contours_color])

        print(f"{color} area :", area_color)

        if area_color > domintant_color_area:
            domintant_color_area = area_color
            dominant_color = color

    return dominant_color


class DiceServer:
    """
    A ROS server to detect the color of a dice from a RGB image.

    The DiceServer class subscribes to the "/rgb/image_raw" topic to receive RGB image messages,
    and provides a service "GetDiceColor" that returns the color of the detected dice.

    Attributes:
        dice_color (str): The color of the detected dice ('green', 'pink', 'yellow', 'blue', 'violet', or 'red').

    Methods:
        image_callback(msg): Callback function to process the received RGB image message.
        dice_color_response(request): Service callback function to respond with the detected dice color.
    """
    def __init__(self):
        rospy.logwarn("Get Color from Dice")
        rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)
        dice_service = rospy.Service("GetDiceColor", GetDiceColor, self.dice_color_response)

    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            # Convert the ROS Image message to a OpenCV image
            dice_image = bridge.imgmsg_to_cv2(msg, "bgr8")

            # Detect the dice and its color
            self.dice_color = detect_dice_color(dice_image)
            print("Dice color:", self.dice_color)

        except Exception as e:
            print(e)

    def dice_color_response(self, request):
        resp = GetDiceColorResponse()
        resp.dice_color = self.dice_color
        return resp


if __name__ == "__main__":
    rospy.init_node("get_dice_color")
    dice_server = DiceServer()
    rospy.spin()
