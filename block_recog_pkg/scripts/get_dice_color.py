#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from block_recog_pkg.srv import GetDiceColor, GetDiceColorResponse


class DiceServer:
    def __init__(self):
        rospy.logwarn("Get Color from Dice")
        rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)
        dice_service = rospy.Service("GetDiceColor", GetDiceColor, self.dice_color_response)

    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            # Convert the ROS Image message to a OpenCV image
            dice_image = bridge.imgmsg_to_cv2(msg, "bgr8")

            # Implement image processing here to detect the dice and its color
            self.dice_color = detect_dice_color(dice_image)
            print("Dice color:", self.dice_color)

        except Exception as e:
            print(e)

    def dice_color_response(self, request):
        resp = GetDiceColorResponse()
        resp.dice_color = self.dice_color
        return resp


def detect_dice_color(img_dice):
    img_hsv = cv2.cvtColor(img_dice, cv2.COLOR_BGR2HSV)  # cvtdice 함수를 이용하여 hsv 색공간으로 변환

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
    lower_green = (70 - 20, 50, 50)
    upper_green = (70 + 15, 255, 255)

    # YELLOW
    lower_yellow = (30 - 10, 80, 80)
    upper_yellow = (30 + 10, 255, 255)

    # BLUE
    lower_blue = (100 - 10, 100, 100)
    upper_blue = (100 + 9, 255, 255)

    # VIOLET
    lower_violet = (130 - 20, 60, 60)
    upper_violet = (130 + 20, 255, 255)

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

        img_mask_color = cv2.inRange(img_hsv, lower_color, upper_color)  # 범위내의 픽셀들은 흰색, 나머지 검은색

        # Find contours of each color's mask
        contours_color, _ = cv2.findContours(img_mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Determine the area of each color's contour
        area_color = sum([cv2.contourArea(cnt) for cnt in contours_color])

        print(f"{color} area :", area_color)

        if area_color > domintant_color_area:
            domintant_color_area = area_color
            dominant_color = color

    return dominant_color


if __name__ == "__main__":
    rospy.init_node("get_dice_color")
    dice_server = DiceServer()
    rospy.spin()
