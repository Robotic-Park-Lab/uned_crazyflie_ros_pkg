from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    dron01_node = Node(
        package='uned_crazyflie_task',
        executable='leader_follower',
        name='LeaderFollower',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'cf_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'cf_num_uri': 1},
            {'cf_control_mode': 'HighLevel'},
            {'cf_controller_type': 'EventBased'},
            {'cf_role': 'leader'},
            {'cf_relationship': 'dron01-dron02'},
        ])
    return LaunchDescription([
        dron01_node
    ])
