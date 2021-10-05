from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uned_crazyflie_driver',
            namespace='dron01',
            executable='crazyflie_driver',
            name='cf_driver'
        ),
        Node(
            package='uned_vicon_bridge',
            executable='vicon_tracker',
            name='vicon_tracker',
            output='screen'
        ),
        Node(
            package='uned_crazyflie_controllers',
            namespace='dron01',
            executable='periodic_pid_position_controller',
            name='position_controller'
        )
    ])
