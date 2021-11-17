from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dron01_node = Node(
        package='uned_crazyflie_driver',
        namespace='dron01',
        executable='crazyflie_driver',
        name='crazyflie',
        output='screen',
        shell=True,
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters=[
            {'cf_uri': 'radio://0/80/2M/E7E7E7E701'}
        ])
    return LaunchDescription([
        dron01_node
    ])
