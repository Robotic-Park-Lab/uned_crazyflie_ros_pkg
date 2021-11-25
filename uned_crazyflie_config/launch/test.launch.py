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
        parameters=[
            {'cf_uri': 'radio://0/80/2M/E7E7E7E701'}
        ])
    vicon_node = Node(
        package='uned_vicon_bridge',
        executable='vicon_tracker',
        name='vicon',
        output='screen',
        shell=True,
        emulate_tty=True,
        )
    dron01_controller = Node(
        package='uned_crazyflie_controllers',
        namespace='dron01',
        executable='periodic_pid_position_controller',
        name='position_pid_controller',
        output='screen',
        shell=True,
        emulate_tty=True
        )
    return LaunchDescription([
        vicon_node,
        dron01_node,
        dron01_controller
    ])
