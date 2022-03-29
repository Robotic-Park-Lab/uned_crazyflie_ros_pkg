from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hostname = '10.196.92.136'
    buffer_size = 200
    topic_namespace = 'vicon'

    swarm_node = Node(
        package='uned_crazyflie_task',
        executable='leader_follower',
        name='LeaderFollower',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'cf_first_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'cf_num_uri': 3},
            {'cf_control_mode': 'HighLevel, HighLevel, HighLevel'},
            {'cf_controller_type': 'EventBased, EventBased, EventBased'},
            {'cf_role': 'leader, leader, follower'},
            {'cf_relationship': 'dron01-dron02, dron02-dron03'},
        ])
    vicon_node = Node(
        package='vicon_receiver',
        executable='vicon_client',
        name='vicon_node',
        parameters=[
            {'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}
        ])
    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface'
    )
    return LaunchDescription([
        swarm_node,
        vicon_node,
        rqt_node
    ])
