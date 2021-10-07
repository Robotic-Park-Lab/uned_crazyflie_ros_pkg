from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uned_crazyflie_controllers',
            executable='periodic_pid_position_controller',
            name='position_controller'
        ),
        Node(
            package='uned_vicon_bridge',
            executable='vicon_tracker',
            name='vicon_tracker'
        ),
        Node(
            package='uned_crazyflie_controllers',
            executable='trajectory_controller',
            name='trajectory_controller'
        ),
        Node(
            package='uned_crazyflie_driver',
            executable='crazyflie_driver',
            name='crazyflie'
        )
    ])
