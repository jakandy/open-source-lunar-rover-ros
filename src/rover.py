#!/usr/bin/env python
#
# Title:
#   Open Source Lunar Rover (oslr) Main
#
# Description:
#   Main program for running the Open Source Lunar Rover using ROS.
#
# Version:
#   v1.0.0, 03/2023
#
# Tested with:
#   ROS Noetic
#   Linux Ubuntu, 20.04 LTS

import include.roverlib as roverlib
import rospy

if __name__ == '__main__':
    rospy.init_node('rover', log_level=rospy.INFO)
    rospy.loginfo("Starting the rover node")
    rover = roverlib.Rover()
    rospy.spin()