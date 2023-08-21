#!/usr/bin/env python3.8

#
# Title:
#   Open Source Lunar Rover (OSLR) Parameters
#
# Author(s):
#   andyjak
#
# Version:
#   0.0.1, 08/2023
#
# Purpose:
#   Generate a yaml file containing parameter values for OSLR.
#
# Notes:
#   This script can be used to tweak the simulation.
#   Usage:
#       1) Change a parameter's value
#       2) Run the script
#       3) Include the exported yaml file in a launch or urdf file
#   New parameters have to be added in the "Export" section.
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

import yaml
import shutil
import os.path as path
import math

# ****************************** PARAMETER VALUES *****************************
# *************** World simulation ***************
# gravity (m/s²)
g_earth = 9.80665
g_moon = 1.625
g_sim = -1 * g_moon

# *************** Mechanical properties ***************
# Rover size dimensions (m) [1]
rover_dimensions = [0.184, 0.267, 0.267, 0.2556]
wheel_radius = 0.075

# Rover mass (kg) [1]
rover_mass = 8.78912

# Rover weight (N)
rover_weight_tot = rover_mass * abs(g_sim)
rover_weight_dist = rover_weight_tot / 6

# rocker-bogie upper and lower limits (rad)
# (this also determines the max tilt of the base_link)
rocker_angle_lim_upper = 0.1
rocker_angle_lim_lower = -1 * rocker_angle_lim_upper

# turning radius upper and lower limits (m) [1]
rover_turning_radius_max = 6.35
rover_turning_radius_min = 0.45

# Wheel-ground interaction values. Assumed to be rubber-dry asphalt. [4],[7]
friction_coeff1_wheel = 0.6
friction_coeff2_wheel = friction_coeff1_wheel
stiffness_wheel = 10000000
damping_wheel = 1.0

# Corner-bogie interaction. Assumed to be aluminium-aluminium. [5],[7]
friction_coeff1_corner = 1.35
friction_coeff2_corner = friction_coeff1_corner
stiffness_corner = 10000000
damping_corner = 1.0

friction_coeff1_bogie = 1.35
friction_coeff2_bogie = friction_coeff1_bogie
stiffness_bogie = 10000000
damping_bogie = 1.0

# Navcam interaction. Assumed to be aluminium-aluminium. [5],[7]
friction_coeff1_navcam_horiz = 1.35
friction_coeff2_navcam_horiz = friction_coeff1_navcam_horiz
stiffness_navcam_horiz = 10000000
damping_navcam_horiz = 1.0

friction_coeff1_navcam_vertical = 1.35
friction_coeff2_navcam_vertical = friction_coeff1_navcam_vertical
stiffness_navcam_vertical = 10000000
damping_navcam_vertical = 1.0

# *************** Joint state controller ***************
# joint state publisher rate (Hz)
# (how often to publish the state of each joint)
joint_state_update_rate = 50

# *************** Steer motor controller ***************
# steer motor PID [24]
kp_steer = 50
ki_steer = 20
kd_steer = 1

# steer motor stall torque (Nm) [3]
steer_stall_torque = 153 * g_earth/100

# effort limit of each steer motor joint (Nm)
steer_effort_lim = steer_stall_torque

# steer motor no-load speed (rpm) [3]
steer_noload_RPM = 45

# steer motor velocity max limit (rad/s)
steer_noload_speed = steer_noload_RPM * 2*math.pi/60
steer_velocity_lim = steer_noload_speed * 0.9

# steer motor angle upper and lower limits (rad) [1]
steer_angle_lim_upper = math.pi/4
steer_angle_lim_lower = -1 * steer_angle_lim_upper

# steer motor mechanical reduction (i.e. gear ratio) [3]
steer_mech_reduction = 189

# minimum steer angle command (rad)
# (smaller angle => smoother turns)
steer_angle_cmd_min = 0.05

# *************** Drive motor controller ***************
# drive motor PID [24]
kp_drive = 2
ki_drive = 10
kd_drive = 0

# drive motor max velocity (rpm) [2]
drive_noload_speed_RPM = 223

# drive motor stall torque (Nm) [2]
drive_stall_torque = 38 * g_earth/100

# effort limit of each drive motor joint (Nm)
drive_effort_lim = drive_stall_torque

# velocity limit of each drive motor joint (rad/s)
drive_noload_speed = drive_noload_speed_RPM * 2*math.pi/60
drive_velocity_lim = drive_noload_speed * 0.9

# drive motor mechanical reduction (i.e. gear ratio) [2]
drive_mech_reduction = 26.9

# *************** Navcam vertical controller ***************
# navcam vertical motor PID [24]
kp_navcam_vertical = 10
ki_navcam_vertical = 0
kd_navcam_vertical = 0.1

# navcam vertical motion effort max limit (Nm)
navcam_vertical_effort_lim = 1

# navcam vertical motion velocity max limit (rad/s)
navcam_vertical_velocity_lim = 1

# navcam vertical motion angle upper and lower limits (rad) [16]
navcam_vertical_angle_lim_lower = -math.pi/2
navcam_vertical_angle_lim_upper = math.pi/2

# *************** Navcam horizontal controller ***************
# navcam horizontal motor PID
kp_navcam_horiz = 10
ki_navcam_horiz = 0
kd_navcam_horiz = 0.25

# navcam horizontal motion effort max limit (Nm)
navcam_horiz_effort_lim = 0.5

# navcam horizontal motion velocity max limit (rad/s)
# (same as MER mast assembly [18])
navcam_horiz_velocity_lim = 0.5 * 2*math.pi/60

# navcam horizontal motion angle upper and lower limits (rad) [18]
navcam_horiz_angle_lim_lower = -math.pi
navcam_horiz_angle_lim_upper = math.pi

# *************** Navcam sensor ***************
# Distance between the lenses (center-to-center) (m) [6]
navcam_baseline = 0.08

# TODO: How often to take images (Hz)
navcam_update_rate = 15

# Field of view (FOV) of camera (rad) [6]
navcam_fov = 83 * 2*math.pi/360

# TODO: Image resolution (px)
navcam_image_width = 400
navcam_image_height = 400

# TODO: Clipping planes (m)
navcam_clip_near = 0.02
navcam_clip_far = 300

# *************** Mechanical limits ***************
# rover velocity max limit (m/s)
rover_velocity_max = wheel_radius * drive_velocity_lim

# ********************************* SUBROUTINES ***********************************

# Subroutine name: writeYamlToFile
# --------------------------------
# Purpose:
#   Writes a python object to a yaml file.
#
# Parameters:
#   py_obj - object to be exported
#   filename - name of the yaml file
#
# Return:
#   None
#
def writeYamlToFile(py_obj,filename):
    with open(f'{filename}.yaml', 'w',) as f :
        f.write('# This file was generated by parameters.py \n\n')
        yaml.dump(py_obj,f,sort_keys=False)

# *********************************** EXPORT **********************************
if __name__ == '__main__':
    data = {
        'steer_angle_lim_upper': steer_angle_lim_upper,
        'steer_angle_lim_lower': steer_angle_lim_lower,
        'steer_effort_lim': steer_effort_lim,
        'steer_velocity_lim': steer_velocity_lim,
        'steer_angle_cmd_min': steer_angle_cmd_min,
        'steer_mech_reduction': steer_mech_reduction,
        'drive_effort_lim': drive_effort_lim,
        'drive_velocity_lim': drive_velocity_lim,
        'drive_mech_reduction': drive_mech_reduction,
        'rover_velocity_max': rover_velocity_max,
        'rover_dimensions': {
            'd1': rover_dimensions[0],
            'd2': rover_dimensions[1],
            'd3': rover_dimensions[2],
            'd4': rover_dimensions[3],
            'wheel_radius': wheel_radius
        },
        'drive_noload_speed_RPM': drive_noload_speed_RPM,
        'joint_state_controller': {
            'type': 'joint_state_controller/JointStateController',
            'publish_rate': joint_state_update_rate
        },
        'front_left_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'front_left_drive_joint',
            'pid': {
                'p': kp_drive,
                'i': ki_drive,
                'd': kd_drive
            }
        },
        'front_right_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'front_right_drive_joint',
            'pid': {
                'p': kp_drive,
                'i': ki_drive,
                'd': kd_drive
            }
        },
        'middle_left_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'middle_left_drive_joint',
            'pid': {
                'p': kp_drive,
                'i': ki_drive,
                'd': kd_drive
            }
        },
        'middle_right_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'middle_right_drive_joint',
            'pid': {
                'p': kp_drive,
                'i': ki_drive,
                'd': kd_drive
            }
        },
        'back_left_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'back_left_drive_joint',
            'pid': {
                'p': kp_drive,
                'i': ki_drive,
                'd': kd_drive
            }
        },
        'back_right_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'back_right_drive_joint',
            'pid': {
                'p': kp_drive,
                'i': ki_drive,
                'd': kd_drive
            }
        },
        'front_left_corner_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'front_left_corner_joint',
            'pid': {
                'p': kp_steer,
                'i': ki_steer,
                'd': kd_steer
            }
        },
        'front_right_corner_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'front_right_corner_joint',
            'pid': {
                'p': kp_steer,
                'i': ki_steer,
                'd': kd_steer
            }
        },
        'back_left_corner_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'back_left_corner_joint',
            'pid': {
                'p': kp_steer,
                'i': ki_steer,
                'd': kd_steer
            }
        },
        'back_right_corner_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'back_right_corner_joint',
            'pid': {
                'p': kp_steer,
                'i': ki_steer,
                'd': kd_steer
            }
        },
        'navcam_horiz_joint_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'navcam_horiz_joint',
            'pid': {
                'p': kp_navcam_horiz,
                'i': ki_navcam_horiz,
                'd': kd_navcam_horiz
            }
        },
        'navcam_vertical_joint_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'navcam_vertical_joint',
            'pid': {
                'p': kp_navcam_vertical,
                'i': ki_navcam_vertical,
                'd': kd_navcam_vertical
            }
        },        
        'friction_coeff1_wheel': friction_coeff1_wheel,
        'friction_coeff2_wheel': friction_coeff2_wheel,
        'stiffness_wheel': stiffness_wheel,
        'damping_wheel': damping_wheel,
        'friction_coeff1_corner': friction_coeff1_corner,
        'friction_coeff2_corner': friction_coeff2_corner,
        'stiffness_corner': stiffness_corner,
        'damping_corner': damping_corner,
        'friction_coeff1_bogie': friction_coeff1_bogie,
        'friction_coeff2_bogie': friction_coeff2_bogie,
        'stiffness_bogie': stiffness_bogie,
        'damping_bogie': damping_bogie,
        'rocker_limit_upper': rocker_angle_lim_upper,
        'rocker_limit_lower': rocker_angle_lim_lower,
        'rover_turning_radius_min': rover_turning_radius_min,
        'rover_turning_radius_max': rover_turning_radius_max,
        'navcam_vertical_angle_lim_upper': navcam_vertical_angle_lim_upper,
        'navcam_vertical_angle_lim_lower': navcam_vertical_angle_lim_lower,
        'navcam_vertical_effort_lim': navcam_vertical_effort_lim,
        'navcam_vertical_velocity_lim': navcam_vertical_velocity_lim,
        'navcam_horiz_angle_lim_upper': navcam_horiz_angle_lim_upper,
        'navcam_horiz_angle_lim_lower': navcam_horiz_angle_lim_lower,
        'navcam_horiz_effort_lim': navcam_horiz_effort_lim,
        'navcam_horiz_velocity_lim': navcam_horiz_velocity_lim,
        'navcam_baseline': navcam_baseline,
        'navcam_update_rate': navcam_update_rate,
        'navcam_fov': navcam_fov,
        'navcam_image_width': navcam_image_width,
        'navcam_image_height': navcam_image_height,
        'navcam_clip_near': navcam_clip_near,
        'navcam_clip_far': navcam_clip_far,
        'friction_coeff1_navcam_horiz': friction_coeff1_navcam_horiz,
        'friction_coeff2_navcam_horiz': friction_coeff2_navcam_horiz,
        'stiffness_navcam_horiz': stiffness_navcam_horiz,
        'damping_navcam_horiz': damping_navcam_horiz,
        'friction_coeff1_navcam_vertical': friction_coeff1_navcam_vertical,
        'friction_coeff2_navcam_vertical': friction_coeff2_navcam_vertical,
        'stiffness_navcam_vertical': stiffness_navcam_vertical,
        'damping_navcam_vertical': damping_navcam_vertical,
        'g_sim': g_sim
    }

    outputFilename = 'oslr_parameters'
    parameterFilePath = './src/oslr_config/config/' + outputFilename

    # Backup the previous parameter file
    if path.exists(parameterFilePath + '.yaml'):
        shutil.copy(parameterFilePath + '.yaml', parameterFilePath + '_old.yaml')
    
    writeYamlToFile(data, parameterFilePath)
    print('Export successful!')