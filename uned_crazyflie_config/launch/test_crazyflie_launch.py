#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_interface = get_package_share_directory('uned_interface')
    pkg_crazyflie_description = get_package_share_directory('uned_crazyflie_description')

    mav_name_arg = DeclareLaunchArgument("mav_name", default_value=TextSubstitution(text="crazyflie2"))
    namespace_arg = DeclareLaunchArgument("namespace", default_value=TextSubstitution(text="crazyflie2"))
    tf_prefix_arg = DeclareLaunchArgument("tf_prefix", default_value=TextSubstitution(text="$(optenv ROS_NAMESPACE)"))
    x_arg = DeclareLaunchArgument("x", default_value=TextSubstitution(text="0.0"))
    y_arg = DeclareLaunchArgument("y", default_value=TextSubstitution(text="0.0"))
    z_arg = DeclareLaunchArgument("z", default_value=TextSubstitution(text="0.0"))
    enable_logging_arg = DeclareLaunchArgument("enable_logging", default_value=TextSubstitution(text="false"))
    enable_ground_truth_arg = DeclareLaunchArgument("enable_ground_truth", default_value=TextSubstitution(text="true"))
    enable_state_estimator_arg = DeclareLaunchArgument("enable_state_estimator", default_value=TextSubstitution(text="false"))
    log_file_arg = DeclareLaunchArgument("log_file", default_value=TextSubstitution(text="crazyflie2"))
    wait_to_record_bag_arg = DeclareLaunchArgument("wait_to_record_bag", default_value=TextSubstitution(text="false"))
    enable_mavlink_interface_arg = DeclareLaunchArgument("enable_mavlink_interface", default_value=TextSubstitution(text="false"))
    enable_mellinger_controller_arg = DeclareLaunchArgument("enable_mellinger_controller", default_value=TextSubstitution(text="false"))
    enable_internal_model_controller_arg = DeclareLaunchArgument("enable_internal_model_controller", default_value=TextSubstitution(text="false"))
    enable_vi_sensor_arg = DeclareLaunchArgument("enable_vi_sensor", default_value=TextSubstitution(text="false"))


    # Start world
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_interface,'launch', 'start_world_launch.py')
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_crazyflie_description,'launch', 'spawn_robot.launch.py')
        )
    )

    return LaunchDescription([
        mav_name_arg,
        namespace_arg,
        tf_prefix_arg,
        x_arg,
        y_arg,
        z_arg,
        enable_logging_arg,
        enable_ground_truth_arg,
        enable_state_estimator_arg,
        log_file_arg,
        wait_to_record_bag_arg,
        enable_mavlink_interface_arg,
        enable_mellinger_controller_arg,
        enable_internal_model_controller_arg,
        enable_vi_sensor_arg,
        start_world,
        spawn_robot_world
    ])
