#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # urdf = os.path.join(get_package_share_directory('pkg_name'), 'dir/', 'flie.urdf')
    # assert os.path.exists(urdf), "msg"

    xacro_file = os.path.join(get_package_share_directory('uned_crazyflie_config'), 'model/urdf/', 'crazyflie2.xacro')
    # xacro_file = '/home/kiko-hp-omen/RoboticPark/src/uned_crazyflie_ros_pkg/uned_crazyflie_config/models/urdf/crazyflie2.xacro'
    assert os.path.exists(xacro_file), "The box_bot.xacro doesnt exist in "+str(xacro_file)
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # print(robot_desc)

    # with open(urdf, 'r') as infp:
    #   robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(package='uned_crazyflie_config', executable='spawn_crazyflie_bot.py', arguments=[robot_desc], output='screen'),
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            name = "robot_state_publisher",
            parameters = [{"robot_description": robot_desc}],
            output = 'screen'
        ),
    ])
