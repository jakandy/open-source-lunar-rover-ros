#!/usr/bin/env python
#
# Title:
#   Open Source Rover (OSR) Parameters
#
# Description:
#   Script for generating a yaml file containing certain parameter values.
#   Its purpose is to centralize all values that can be set to tweak
#   parts of the model simulation (see Parameter values section).
#
# Version:
#   v1.0.0, 03/2023
#
# Tested with:
#   ROS Noetic
#   Linux Ubuntu, 20.04 LTS
#
# References:
#   - Open Source Rover: Software Controls (pdf)
#   - corner-motor-datasheet (pdf)
#   - drive-motor-datasheet (pdf)
#

import yaml
import shutil
import os.path as path
import math

# *****************************************************************************
# ****************************** PARAMETER VALUES *****************************
# *****************************************************************************
# World simulation
g_std = 9.80665                         # standard gravity acceleration [m/s²]
g = -g_std                              # simulated gravity [m/s²]
friction_coeff_1 = "0.9"
friction_coeff_2 = friction_coeff_1
stiffness_ground_wheel = "10000000.0"
damping_ground_wheel = "1.0"

# Joint controllers
kp_wheel = 2
ki_wheel = 10
kd_wheel = 0

kp_steer = 50
ki_steer = 20
kd_steer = 1

joint_state_update_rate = 50                # [Hz]

# Geometric dimensions
rover_dimensions = [0.184,
                    0.267,
                    0.267,
                    0.2556]                 # [m]
wheel_radius_value = 0.075                  # [m]

# Physical limits to locomotion
motor_steer_effort_max = 153 * g_std/100    # [Nm]
motor_steer_vel_max = 45 * 2*math.pi/60     # [rad/s]
motor_steer_angle_max = math.pi/4           # [rad]
motor_steer_angle_min = -1 * motor_steer_angle_max # [rad]
motor_drive_noload_value_rpm = 223          # [rpm]
motor_drive_effort_max = 38 * g_std/100     # [Nm]
motor_drive_vel_max = motor_drive_noload_value_rpm * 2*math.pi/60 # [rad/s]

# *****************************************************************************
# ********************************* OBJECTS ***********************************
# *****************************************************************************
# Subroutine name: writeYamlToFile
# --------------------------------
# Description:
#   Writes a python object to a yaml file
#
# Input parameters:
#   py_obj - object to be exported
#   filename - name of the yaml file without extension
#
# Return:
#   ...
#
def writeYamlToFile(py_obj,filename):
    with open(f'{filename}.yaml', 'w',) as f :
        yaml.dump(py_obj,f,sort_keys=False) 

# *****************************************************************************
# ************************************ MAIN ***********************************
# *****************************************************************************
if __name__ == '__main__':
    data = {
        'steer_angle_max': motor_steer_angle_max,
        'steer_angle_min': motor_steer_angle_min,
        'steer_effort_max': motor_steer_effort_max,
        'steer_vel_max': motor_steer_vel_max,
        'drive_effort_max': motor_drive_effort_max,
        'drive_vel_max': motor_drive_vel_max,
        'rover_dimensions': {
            'd1': rover_dimensions[0],
            'd2': rover_dimensions[1],
            'd3': rover_dimensions[2],
            'd4': rover_dimensions[3],
            'wheel_radius': wheel_radius_value
        },
        'motor_drive_noload_rpm': motor_drive_noload_value_rpm,
        'joint_state_controller': {
            'type': 'joint_state_controller/JointStateController',
            'publish_rate': joint_state_update_rate
        },
        'front_left_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'front_left_drive_joint',
            'pid': {
                'p': kp_wheel, 'i': ki_wheel, 'd': kd_wheel
            }
        },
        'front_right_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'front_right_drive_joint',
            'pid': {
                'p': kp_wheel, 'i': ki_wheel, 'd': kd_wheel
            }
        },
        'middle_left_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'middle_left_drive_joint',
            'pid': {
                'p': kp_wheel, 'i': ki_wheel, 'd': kd_wheel
            }
        },
        'middle_right_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'middle_right_drive_joint',
            'pid': {
                'p': kp_wheel, 'i': ki_wheel, 'd': kd_wheel
            }
        },
        'back_left_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'back_left_drive_joint',
            'pid': {
                'p': kp_wheel, 'i': ki_wheel, 'd': kd_wheel
            }
        },
        'back_right_wheel_controller': {
            'type': 'effort_controllers/JointVelocityController',
            'joint': 'back_right_drive_joint',
            'pid': {
                'p': kp_wheel, 'i': ki_wheel, 'd': kd_wheel
            }
        },
        'front_left_corner_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'front_left_corner_joint',
            'pid': {
                'p': kp_steer, 'i': ki_steer, 'd': kd_steer
            }
        },
        'front_right_corner_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'front_right_corner_joint',
            'pid': {
                'p': kp_steer, 'i': ki_steer, 'd': kd_steer
            }
        },
        'back_left_corner_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'back_left_corner_joint',
            'pid': {
                'p': kp_steer, 'i': ki_steer, 'd': kd_steer
            }
        },
        'back_right_corner_controller': {
            'type': 'effort_controllers/JointPositionController',
            'joint': 'back_right_corner_joint',
            'pid': {
                'p': kp_steer, 'i': ki_steer, 'd': kd_steer
            }
        },
        'friction_1': friction_coeff_1,
        'friction_2': friction_coeff_2,
        'stiffness': stiffness_ground_wheel,
        'damping': damping_ground_wheel
    }

    outputFilename = 'osr_parameters'
    parameterFilePath = './src/osr/config/' + outputFilename

    # Backup the previous parameter file before writing a new one
    if path.exists(parameterFilePath + '.yaml'):
        shutil.copy(parameterFilePath + '.yaml', parameterFilePath + '_old.yaml')
    
    writeYamlToFile(data, parameterFilePath)
    print('Export successful!')