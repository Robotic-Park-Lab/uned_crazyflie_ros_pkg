import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_crazyflie_config')
    config_path = os.path.join(config_package_dir, 'resources', 'swarm_teleop.yaml')
    # config_path = os.path.join(config_package_dir, 'resources', 'swarm_formation_distance.yaml')
    
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')

    swarm_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'config': config_path},
            {'robots': 'dron01'}
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
