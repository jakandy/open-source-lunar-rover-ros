#!/usr/bin/env python3.8

#
# Title:
#   Open Source Lunar Rover (OSLR) - Navcam Main
#
# Author(s):
#   andyjak
#
# Version:
#   0.0.1, 08/2023
#
# Purpose:
#   Start the navcam node.
#
# Notes:
#   This script enables the rover's navcam image processing.
#
# Test setup:
#   - ROS Noetic
#   - Ubuntu, 20.04 LTS
#   - Python 3.8.10
#
# References:
#   References are listed in the README.
#   Comments that include a [number] refers to the index in the reference list.
#

import rospy
import message_filters
import actionlib

from sensor_msgs.msg import Image, CameraInfo
from oslr_msgs.msg import stereo_depthAction, stereo_depthGoal

# ********************************* OBJECTS ***********************************
class Navcam_main(object):
    
    # Subroutine name: init
    # --------------------------------
    # Purpose:
    #   Initializes the object.
    #
    # Notes:
    #   This is the first function that is called when the object is created.
    #   It sets up all nodes used by the rover.
    #
    # Parameters:
    #   None
    #
    # Return:
    #   None
    #
    def __init__(self):
        # Subscribe to camera topics
        navcam_left_sub = message_filters.Subscriber("/oslr/navcam/left/image_raw", Image)
        navcam_right_sub = message_filters.Subscriber("/oslr/navcam/right/image_raw", Image)
        navcam_info_sub = message_filters.Subscriber("/oslr/navcam/left/camera_info", CameraInfo)

        # Merge camera topics into one
        navcam_stereo_msg = message_filters.TimeSynchronizer([navcam_left_sub, navcam_right_sub, navcam_info_sub], 1)
        navcam_stereo_msg.registerCallback(self.navcam_cb)

        # Initialize action client
        self.navcam_stereo_depth_client = actionlib.SimpleActionClient('/navcam_depth_server', stereo_depthAction)

    # Subroutine name: navcam_cb
    # --------------------------------
    # Purpose:
    #   Carry out the commands of the action.
    #
    # Notes:
    #   This is a callback function that runs every time camera messages
    #   are received. 
    #
    # Parameters:
    #   image_left: image message from left camera
    #   image_right: image message from right camera
    #   camera_info: timestamps and metadata of camera message
    #
    # Return:
    #   None
    #
    def navcam_cb(self, image_left, image_right, camera_info):
        # waits until the action server is up and running
        self.navcam_stereo_depth_client.wait_for_server()

        # creates a goal and send it to the action server
        goal = stereo_depthGoal()
        goal.left = image_left
        goal.right = image_right
        goal.camera_info = camera_info
        self.navcam_stereo_depth_client.send_goal(goal)

        # Wait until action is done so that rover does not act
        # on old data if a camera goes down.
        self.navcam_stereo_depth_client.wait_for_result()
        # TODO: add timeout routine
        return
    
# ********************************** MAIN ************************************
if __name__ == '__main__':
    rospy.init_node('navcam', log_level=rospy.INFO)
    rospy.loginfo("Starting navcam")
    navcam = Navcam_main()
    rospy.spin()