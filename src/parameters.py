#!/usr/bin/env python
#
# Title:
#   Open Source Rover (OSR) Constants
#
# Description:
#   Script for easily being able to change parameters used in other files.
#   Change the value you want here, run the script and the corresponding
#   value will change.
#
# Version:
#   v1.0.0, 03/2023
#
# Tested with:
#   ROS Noetic
#   Linux Ubuntu, 20.04 LTS
#

import yaml

def write_yaml_to_file(py_obj,filename):
    with open(f'{filename}.yaml', 'w',) as f :
        yaml.dump(py_obj,f,sort_keys=False) 

kp_wheel = 1
ki_wheel = 10
kd_wheel = 0

if __name__ == '__main__':
    data = {
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
        }
    }


    # yaml_output = yaml.dump(data, sort_keys=False) 
    # print(yaml_output)

    write_yaml_to_file(data, './src/osr/config/osr_parameters')
    print('Export successful!')