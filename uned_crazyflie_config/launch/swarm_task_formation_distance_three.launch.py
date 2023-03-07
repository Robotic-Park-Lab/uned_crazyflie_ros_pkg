import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_crazyflie_config')
    config_path = os.path.join(config_package_dir, 'resource', 'demo_swarm_formation_distance_three.yaml')
    # config_path = os.path.join(config_package_dir, 'resource', 'crazyflie_ros2_formation_distance_three_event.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')

    swarm_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'first_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'n': 3},
            {'config': config_path}
        ]
    )

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        swarm_node,
        rqt_node,
        rviz_node
    ])
