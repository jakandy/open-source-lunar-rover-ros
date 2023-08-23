#!/usr/bin/env python3.8

#
# Title:
#   Open Source Lunar Rover (OSLR) - Motion control
#
# Author(s):
#   andyjak
#
# Version:
#   0.0.1, 08/2023
#
# Purpose:
#   Start all the nodes responsible for control of the rover.
#
# Notes:
#   This script enables the rover's actuators and sensors to be controlled.
#   It can be started using rosrun or from a launch file.
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
import math
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
from oslr_msgs.msg import Drive, Steer
from std_msgs.msg import Float64

# ********************************* OBJECTS ***********************************
class Motion(object):
    
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
        # Get parameter values
        rover_dimensions = rospy.get_param('/rover_dimensions')
        self.d1 = rover_dimensions["d1"]
        self.d2 = rover_dimensions["d2"]
        self.d3 = rover_dimensions["d3"]
        self.d4 = rover_dimensions["d4"]

        self.min_radius = rospy.get_param("/rover_turning_radius_min")
        self.max_radius = rospy.get_param("/rover_turning_radius_max")

        self.wheel_radius = rospy.get_param("/rover_dimensions/wheel_radius")

        self.max_vel = rospy.get_param("/rover_velocity_max")

        self.should_calculate_odom = rospy.get_param("~enable_odometry", False)

        self.steer_angle_cmd_min = rospy.get_param("/steer_angle_cmd_min")

        # Initialize twist message
        self.curr_twist = TwistWithCovariance()
        self.curr_turning_radius = self.max_radius

        # Initialize steer motors
        self.corner_left_front_cmd_pub = rospy.Publisher("/front_left_corner_controller/command", Float64, queue_size=1)
        self.corner_right_front_cmd_pub = rospy.Publisher("/front_right_corner_controller/command", Float64, queue_size=1)
        self.corner_left_back_cmd_pub = rospy.Publisher("/back_left_corner_controller/command", Float64, queue_size=1)
        self.corner_right_back_cmd_pub = rospy.Publisher("/back_right_corner_controller/command", Float64, queue_size=1)

        # Initialize drive motors
        self.drive_left_front_cmd_pub = rospy.Publisher("/front_left_wheel_controller/command", Float64, queue_size=1)
        self.drive_left_middle_cmd_pub = rospy.Publisher("/middle_left_wheel_controller/command", Float64, queue_size=1)
        self.drive_left_back_cmd_pub = rospy.Publisher("/back_left_wheel_controller/command", Float64, queue_size=1)
        self.drive_right_front_cmd_pub= rospy.Publisher("/front_right_wheel_controller/command", Float64, queue_size=1)
        self.drive_right_middle_cmd_pub = rospy.Publisher("/middle_right_wheel_controller/command", Float64, queue_size=1)
        self.drive_right_back_cmd_pub = rospy.Publisher("/back_right_wheel_controller/command", Float64, queue_size=1)

        # Initialize motor sensors
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)
        rospy.Subscriber("/joint_states", JointState, self.enc_cb)
        self.odometry = Odometry()
        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = "odom"
        self.odometry.child_frame_id = "base_link"
        self.odometry.pose.pose.orientation.z = 0.
        self.odometry.pose.pose.orientation.w = 1.
        if self.should_calculate_odom:
            self.odometry_pub = rospy.Publisher("/odom", Odometry, queue_size=2)
        self.tf_pub = tf2_ros.TransformBroadcaster()

    # Subroutine name: cmd_cb
    # --------------------------------
    # Purpose:
    #   Commands the rover's motors.
    #
    # Notes:
    #   This is a callback function that runs every time a twist message
    #   is received on the cmd_vel topic. The position and velocity of the wheels
    #   is calculated and then published to each motor's corresponding controller.
    #
    # Parameters:
    #   twist_msg - twist message from user
    #
    # Return:
    #   None
    #
    def cmd_cb(self, twist_msg):
        # calculate the position of each wheel
        desired_turning_radius = self.twist_to_turning_radius(twist_msg)
        corner_cmd_msg = self.calculate_corner_positions(desired_turning_radius)

        # if turning, set the max velocity to 50%
        max_vel = abs(desired_turning_radius) / (abs(desired_turning_radius) + self.d1) * self.max_vel
        if math.isnan(max_vel):
            # rover is travelling straight
            max_vel = self.max_vel
        velocity = min(max_vel, twist_msg.linear.x)

        # Calculate velocity for each motor
        drive_cmd_msg = self.calculate_drive_velocities(velocity, desired_turning_radius)

        # Publish position commands for each corner motor
        if self.corner_cmd_threshold(corner_cmd_msg):
           self.corner_left_front_cmd_pub.publish(corner_cmd_msg.left_front_pos)
           self.corner_right_front_cmd_pub.publish(corner_cmd_msg.right_front_pos)
           self.corner_left_back_cmd_pub.publish(corner_cmd_msg.left_back_pos)
           self.corner_right_back_cmd_pub.publish(corner_cmd_msg.right_back_pos)
        
        # Publish velocity commands for each drive motor
        self.drive_left_front_cmd_pub.publish(drive_cmd_msg.left_front_vel)
        self.drive_left_middle_cmd_pub.publish(drive_cmd_msg.left_middle_vel)
        self.drive_left_back_cmd_pub.publish(drive_cmd_msg.left_back_vel)
        self.drive_right_front_cmd_pub.publish(drive_cmd_msg.right_front_vel)
        self.drive_right_middle_cmd_pub.publish(drive_cmd_msg.right_middle_vel)
        self.drive_right_back_cmd_pub.publish(drive_cmd_msg.right_back_vel)
        return

    # Subroutine name: enc_cb
    # --------------------------------
    # Purpose:
    #   Determines the position, velocity and effort of each joint of the rover.
    #
    # Notes:
    #   Callback function for encoder topic.
    #
    # Parameters:
    #   msg - JointState message
    #
    # Return:
    #   None
    #
    def enc_cb(self, msg):
        self.curr_positions = dict(zip(msg.name, msg.position))
        self.curr_velocities = dict(zip(msg.name, msg.velocity))
        # TODO: move constants to parameter file
        if self.should_calculate_odom:
            # measure how much time has elapsed since our last update
            now = rospy.Time.now()
            dt = (now - self.odometry.header.stamp).to_sec()
            self.forward_kinematics()
            dx = self.curr_twist.twist.linear.x * dt
            dth = self.curr_twist.twist.angular.z * dt

            # angle is straightforward: in 2D it's additive
            # first calculate the current_angle in the fixed frame
            current_angle = 2 * math.atan2(self.odometry.pose.pose.orientation.z, 
                                           self.odometry.pose.pose.orientation.w)
            new_angle = current_angle + dth
            self.odometry.pose.pose.orientation.z = math.sin(new_angle/2.)
            self.odometry.pose.pose.orientation.w = math.cos(new_angle/2.)

            # the new pose in x and y depends on the current heading
            self.odometry.pose.pose.position.x += math.cos(new_angle) * dx
            self.odometry.pose.pose.position.y += math.sin(new_angle) * dx
            self.odometry.pose.covariance = 36 * [0.0,]
            self.odometry.twist.covariance = 36 * [0.0,]
            
            # explanation for values at [28]
            self.odometry.twist.covariance[0] = 0.0225
            self.odometry.twist.covariance[5] = 0.01
            self.odometry.twist.covariance[-5] = 0.0225
            self.odometry.twist.covariance[-1] = 0.04
            self.odometry.twist = self.curr_twist
            self.odometry.header.stamp = now
            self.odometry_pub.publish(self.odometry)
            transform_msg = TransformStamped()
            transform_msg.header.frame_id = "odom"
            transform_msg.child_frame_id = "base_link"
            transform_msg.header.stamp = now
            transform_msg.transform.translation.x = self.odometry.pose.pose.position.x
            transform_msg.transform.translation.y = self.odometry.pose.pose.position.y
            transform_msg.transform.rotation = self.odometry.pose.pose.orientation
            self.tf_pub.sendTransform(transform_msg)
        return
    
    # Subroutine name: corner_cmd_threshold
    # --------------------------------
    # Purpose:
    #   Determines if the command to corner motors are above a certain
    #   threshold angle in radians.
    #
    # Notes:
    #   The threshold angle can be set in parameters.py.
    #
    # Parameters:
    #   corner_cmd - Steer message
    #
    # Return:
    #   True (1) - if the angle is above the threshold
    #   False (0) - if the angle is below the threshold
    #
    def corner_cmd_threshold(self, corner_cmd):
        try:
            # encoder positions have been received
            if abs(corner_cmd.left_front_pos - self.curr_positions["front_left_corner_joint"]) > self.steer_angle_cmd_min:
                return True
            elif abs(corner_cmd.left_back_pos - self.curr_positions["back_left_corner_joint"]) > self.steer_angle_cmd_min:
                return True
            elif abs(corner_cmd.right_back_pos - self.curr_positions["back_right_corner_joint"]) > self.steer_angle_cmd_min:
                return True
            elif abs(corner_cmd.right_front_pos - self.curr_positions["front_right_corner_joint"]) > self.steer_angle_cmd_min:
                return True
            else:
                return False
        except AttributeError:
            # haven't received current encoder positions yet
            return True

    # Subroutine name: calculate_drive_velocities
    # --------------------------------
    # Purpose:
    #   Calculate target velocities for the drive motors.
    #
    # Notes:
    #   The calculations are based on desired speed and current turning radius.
    #
    # Parameters:
    #   speed - Drive speed command range from -max_vel to max_vel, with max vel depending on the turning radius
    #   current_radius - Current turning radius in m
    #
    # Return:
    #   cmd_msg - Drive message
    #
    def calculate_drive_velocities(self, speed, current_radius):
        # clip the value to the maximum allowed velocity
        speed = max(-self.max_vel, min(self.max_vel, speed))
        cmd_msg = Drive()
        if speed == 0:
            return cmd_msg

        elif abs(current_radius) >= self.max_radius:  # Very large turning radius, all wheels same speed
            angular_vel = speed / self.wheel_radius
            cmd_msg.left_front_vel = angular_vel
            cmd_msg.left_middle_vel = angular_vel
            cmd_msg.left_back_vel = angular_vel
            cmd_msg.right_back_vel = angular_vel
            cmd_msg.right_middle_vel = angular_vel
            cmd_msg.right_front_vel = angular_vel

            return cmd_msg

        else:
            # for the calculations, we assume positive radius (turn left) and adjust later
            radius = abs(current_radius)
            # the entire vehicle moves with the same angular velocity dictated by the desired speed,
            # around the radius of the turn. v = r * omega
            angular_velocity_center = float(speed) / radius
            # calculate desired velocities of all centers of wheels. Corner wheels on the same side
            # move with the same velocity. v = r * omega again
            vel_middle_closest = (radius - self.d4) * angular_velocity_center
            vel_corner_closest = math.hypot(radius - self.d1, self.d3) * angular_velocity_center
            vel_corner_farthest = math.hypot(radius + self.d1, self.d3) * angular_velocity_center
            vel_middle_farthest = (radius + self.d4) * angular_velocity_center

            # now from these desired velocities, calculate the desired angular velocity of each wheel
            # v = r * omega again
            ang_vel_middle_closest = vel_middle_closest / self.wheel_radius
            ang_vel_corner_closest = vel_corner_closest / self.wheel_radius
            ang_vel_corner_farthest = vel_corner_farthest / self.wheel_radius
            ang_vel_middle_farthest = vel_middle_farthest / self.wheel_radius

            if current_radius > 0:  # turning left
                cmd_msg.left_front_vel = ang_vel_corner_closest
                cmd_msg.left_back_vel = ang_vel_corner_closest
                cmd_msg.left_middle_vel = ang_vel_middle_closest
                cmd_msg.right_back_vel = ang_vel_corner_farthest
                cmd_msg.right_front_vel = ang_vel_corner_farthest
                cmd_msg.right_middle_vel = ang_vel_middle_farthest
            else:  # turning right
                cmd_msg.left_front_vel = ang_vel_corner_farthest
                cmd_msg.left_back_vel = ang_vel_corner_farthest
                cmd_msg.left_middle_vel = ang_vel_middle_farthest
                cmd_msg.right_back_vel = ang_vel_corner_closest
                cmd_msg.right_front_vel = ang_vel_corner_closest
                cmd_msg.right_middle_vel = ang_vel_middle_closest

            return cmd_msg

    # Subroutine name: calculate_corner_positions
    # --------------------------------
    # Purpose:
    #   Takes a turning radius and computes the required angle for each corner motor.
    #
    # Notes:
    #   A small turning radius means a sharp turn
    #   A large turning radius means mostly straight. Any radius larger than max_radius is essentially straight
    #   because of the encoders' resolution.
    #   The positions are expressed in the motor's frame with the positive z-axis pointing down. This means
    #   that a positive angle corresponds to a right turn
    #
    # Parameters:
    #   radius - positive value means turn left.
    #
    # Return:
    #   cmd_msg - Steer message
    #
    def calculate_corner_positions(self, radius):
        cmd_msg = Steer()

        if radius >= self.max_radius:
            return cmd_msg

        theta_front_closest = math.atan2(self.d3, abs(radius) - self.d1)
        theta_front_farthest = math.atan2(self.d3, abs(radius) + self.d1)

        if radius > 0:
            cmd_msg.left_front_pos = -theta_front_closest
            cmd_msg.left_back_pos = theta_front_closest
            cmd_msg.right_back_pos = theta_front_farthest
            cmd_msg.right_front_pos = -theta_front_farthest
        else:
            cmd_msg.left_front_pos = theta_front_farthest
            cmd_msg.left_back_pos = -theta_front_farthest
            cmd_msg.right_back_pos = -theta_front_closest
            cmd_msg.right_front_pos = theta_front_closest
        
        return cmd_msg

    # Subroutine name: twist_to_turning_radius
    # --------------------------------
    # Purpose:
    #   Convert a commanded twist into an actual turning radius
    #
    # Notes:
    #   See [1]
    #
    # Parameters:
    #   twist - geometry_msgs/Twist. Only linear.x and angular.z are used
    #   clip - whether the values should be clipped from min_radius to max_radius
    #
    # Return:
    #   radius - physical turning radius in meters, clipped to the rover's limits
    #
    def twist_to_turning_radius(self, twist, clip=True):
        try:
            if twist.linear.x < 0:
                radius = twist.linear.x / -twist.angular.z
            else:
                radius = twist.linear.x / twist.angular.z
        except ZeroDivisionError:
                return float("Inf")

        # clip values so they lie in (-max_radius, -min_radius) or (min_radius, max_radius)
        if not clip:
            return radius
        if radius == 0:
            return self.max_radius  
        if radius > 0:
            radius = max(self.min_radius, min(self.max_radius, radius))
        else:
            radius = max(-self.max_radius, min(-self.min_radius, radius))

        return radius

    # Subroutine name: angle_to_turning_radius
    # --------------------------------
    # Purpose:
    #   Convert the angle of a virtual wheel positioned in the middle of
    #   the front two wheels to a turning radius.
    #
    # Notes:
    #   Turning left and positive angle corresponds to a positive turning radius.
    #
    # Parameters:
    #   angle - angle of each wheel [rad]
    #
    # Return:
    #   radius - turning radius for the given angle in [m]
    #
    def angle_to_turning_radius(self, angle):
        try:
            radius = self.d3 / math.tan(angle)
        except ZeroDivisionError:
            return float("Inf")

        return radius

    # Subroutine name: forward_kinematics
    # --------------------------------
    # Purpose:
    #   Calculate current twist of the rover given current drive and corner motor velocities
    #
    # Notes:
    #   Note that forward kinematics means solving an overconstrained system since the corner 
    #   motors may not be aligned perfectly and drive velocities might fight each other
    #
    # Parameters:
    #   None
    #
    # Return:
    #   None
    #
    def forward_kinematics(self):
        # calculate current turning radius according to each corner wheel's angle
        # corner motor angles should be flipped since different coordinate axes in this node (positive z up)
        theta_fl = -self.curr_positions['front_left_corner_joint']
        theta_fr = -self.curr_positions['front_right_corner_joint']
        theta_bl = -self.curr_positions['back_left_corner_joint']
        theta_br = -self.curr_positions['back_right_corner_joint']
        # sum wheel angles to find out which direction the rover is mostly turning in
        if theta_fl + theta_fr + theta_bl + theta_br > 0:  # turning left
            r_front_closest = self.d1 + self.angle_to_turning_radius(theta_fl)
            r_front_farthest = -self.d1 + self.angle_to_turning_radius(theta_fr)
            r_back_closest = -self.d1 - self.angle_to_turning_radius(theta_bl)
            r_back_farthest = self.d1 - self.angle_to_turning_radius(theta_br)
        else:  # turning right
            r_front_farthest = self.d1 + self.angle_to_turning_radius(theta_fl)
            r_front_closest = -self.d1 + self.angle_to_turning_radius(theta_fr)
            r_back_farthest = -self.d1 - self.angle_to_turning_radius(theta_bl)
            r_back_closest = self.d1 - self.angle_to_turning_radius(theta_br)
        # get a best estimate of the turning radius by taking the median value (avg sensitive to outliers)
        approx_turning_radius = sum(sorted([r_front_farthest, r_front_closest, r_back_farthest, r_back_closest])[1:3])/2.0
        if math.isnan(approx_turning_radius):
            approx_turning_radius = self.max_radius
        self.curr_turning_radius = approx_turning_radius

        # we know that the linear velocity in x direction is the instantaneous velocity of the middle virtual
        # wheel which spins at the average speed of the two middle outer wheels.
        drive_angular_velocity = (self.curr_velocities['middle_left_drive_joint'] + self.curr_velocities['middle_right_drive_joint']) / 2.
        self.curr_twist.twist.linear.x = drive_angular_velocity * self.wheel_radius
        # now calculate angular velocity from its relation with linear velocity and turning radius
        try:
            self.curr_twist.twist.angular.z = self.curr_twist.twist.linear.x / self.curr_turning_radius
        except ZeroDivisionError:
            rospy.logwarn_throttle(1, "Current turning radius was calculated as zero which"
                                   "is an illegal value. Check your wheel calibration.")
            self.curr_twist.twist.angular.z = 0.  # turning in place is currently unsupported
        return

# ********************************** MAIN ************************************
if __name__ == '__main__':
    rospy.init_node('motion_control', log_level=rospy.INFO)
    rospy.loginfo("Starting motion control")
    motion_control = Motion()
    rospy.spin()