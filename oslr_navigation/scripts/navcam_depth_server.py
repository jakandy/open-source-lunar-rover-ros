#!/usr/bin/env python3.8

#
# Title:
#   Open Source Lunar Rover (OSLR) - Navcam Depth Server
#
# Author(s):
#   andyjak
#
# Version:
#   0.0.1, 08/2023
#
# Purpose:
#   Calculate the distance to surrounding objects from stereo camera images.
#
# Notes:
#   For more information about depth procesing see [19].
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
import actionlib
from oslr_msgs.msg import stereo_depthAction, stereo_depthGoal
import cv2 as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

# ********************************* OBJECTS ***********************************
class Action_object(object):
    
    # Subroutine name: init
    # --------------------------------
    # Purpose:
    #   Initializes the object.
    #
    # Notes:
    #   This is the first function that is called when the object is created.
    #
    # Parameters:
    #   None
    #
    # Return:
    #   None
    #
    def __init__(self):
        # Initialize the action server
        self.server = actionlib.SimpleActionServer("/navcam_depth_server", stereo_depthAction, self.execute_action, False)
        
        # Establish a bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # Start the action server
        self.server.start()
        return

    # Subroutine name: execute_action
    # --------------------------------
    # Purpose:
    #   Carry out the commands of the action.
    #
    # Notes:
    #   This function gets called after a client sends a request.
    #
    # Parameters:
    #   goal - defined in action file
    #
    # Return:
    #   None
    #
    def execute_action(self, goal):
        # Convert ROS image message to OpenCv image [25]
        cv_image_left = self.bridge.imgmsg_to_cv2(goal.left, desired_encoding="mono8")
        cv_image_right = self.bridge.imgmsg_to_cv2(goal.right, desired_encoding="mono8")

        # Create disparity map [26]
        # numDisparities must be positive and divisible by 16. The larger it is, the larger the range of visible depths.
        # blockSize must be odd. speckle_size is the number of pixels below which a disparity blob is dismissed as "speckle."
        stereo = cv.StereoBM.create(numDisparities=32, blockSize=7)
        disparity_map = stereo.compute(cv_image_left, cv_image_right)
        img8 = disparity_map.astype('uint8')

        # # Image preview
        plt.imshow(disparity_map, 'gray')
        plt.show()
        # TODO: turn disparity map into pointcloud
        
        # Send done command
        self.server.set_succeeded()
        return

# ********************************** MAIN ************************************
if __name__ == '__main__':
    rospy.init_node('navcam_depth_server', log_level=rospy.INFO)
    stereo_depth_server = Action_object()
    rospy.spin()