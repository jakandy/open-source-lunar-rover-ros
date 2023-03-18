#!/usr/bin/env python
#
# Title:
#   Open Source Rover (OSR) Parameters
#
# Description:
#   Script for easily being able to change parameters used in other files.
#   Change the value you want here, run the script and the corresponding
#   value will change.
#   The values get exported to a yaml file that can be loaded onto the
#   parameter server.
#
# Version:
#   v1.0.0, 03/2023
#
# Tested with:
#   ROS Noetic
#   Linux Ubuntu, 20.04 LTS
#

import yaml
import shutil
import os.path as path

# *****************************************************************************
# ****************************** PARAMETER VALUES *****************************
# *****************************************************************************
kp_wheel = 1
ki_wheel = 10
kd_wheel = 0

kp_steer = 50
ki_steer = 20
kd_steer = 1

joint_state_update_rate = 50

rover_dimensions = [0.184, 0.267, 0.267, 0.2556]
wheel_radius_value = 0.075

motor_drive_rpm_noload_value = 223

gravity = -9.8

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
        'rover_dimensions': {
            'd1': rover_dimensions[0],
            'd2': rover_dimensions[1],
            'd3': rover_dimensions[2],
            'd4': rover_dimensions[3],
            'wheel_radius': wheel_radius_value
        },
        'motor_drive_rpm_noload': motor_drive_rpm_noload_value,
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
        }
    }

    parameterFileName = 'osr_parameters'
    parameterFilePath = './src/osr/config/' + parameterFileName

    # Backup the previous parameter file before writing a new one
    if path.exists(parameterFilePath + '.yaml'):
        shutil.copy(parameterFilePath + '.yaml', parameterFilePath + '_old.yaml')
    
    writeYamlToFile(data, parameterFilePath)
    print('Export successful!')