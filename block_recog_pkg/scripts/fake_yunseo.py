#!/usr/bin/env python

import rospy
from block_recog_pkg.srv import GetWorldCoord, GetWorldCoordRequest, GetWorldCoordResponse, CaptureImage, CaptureImageRequest, CaptureImageResponse

rospy.init_node("service_client")

rospy.wait_for_service('CaptureImage')

capture_image = rospy.ServiceProxy('CaptureImage', CaptureImage)

request_capture_image = CaptureImageRequest()

response = capture_image(request_capture_image)

if response.status == response.FAILED:
    rospy.logwarn("Failed to Capture Image")
elif response.status == response.SUCCESS:
    rospy.loginfo("Image Captured")
elif response.status == response.SKIPPED:
    rospy.loginfo("Image Capture Skipped")

rospy.wait_for_service('GetWorldCoordinates')

get_coord = rospy.ServiceProxy('GetWorldCoordinates', GetWorldCoord)

request = GetWorldCoordRequest()

request.target_block = "init 1"

response = get_coord(request.target_block)

print(response.success)
print(response.center_x)
print(response.center_y)
print(response.center_z)
print(response.target_x)
print(response.target_y)
print(response.target_z)
print(response.push)

rospy.spin()