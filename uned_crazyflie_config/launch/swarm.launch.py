from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    dron01_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'cf_first_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'cf_num_uri': 3},
            {'cf_control_mode': 'HighLevel, HighLevel, HighLevel, HighLevel'},
            {'cf_controller_type': 'EventBased, Continuous, Continuous, Continuous'}
        ])
    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface'
    )
    return LaunchDescription([
        dron01_node,
        rqt_node
    ])
