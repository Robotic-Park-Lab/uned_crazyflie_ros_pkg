from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uned_vicon_bridge',
            executable='vicon_tracker',
            name='vicon_tracker'
        ),
        Node(
            package='uned_crazyflie_driver',
            namespace='dron01',
            executable='crazyflie_driver',
            name='crazyflie',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'cf_uri': 'radio://0/80/2M/E7E7E7E7E7'}
            ]
        )
    ])
