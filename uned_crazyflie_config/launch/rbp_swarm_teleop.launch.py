import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_crazyflie_config')
    config_path = os.path.join(config_package_dir, 'resources', 'swarm_teleop.yaml')
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
            {'robots': 'dron02, dron03'}
        ]
    )

    return LaunchDescription([
        swarm_node
    ])
