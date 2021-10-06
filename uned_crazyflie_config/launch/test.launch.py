from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uned_crazyflie_controllers',
            namespace='dron01',
            executable='periodic_pid_position_controller',
            name='position_controller'
        ),
        Node(
            package='uned_vicon_bridge',
            executable='vicon_tracker',
            name='vicon_tracker'
        )
    ])
