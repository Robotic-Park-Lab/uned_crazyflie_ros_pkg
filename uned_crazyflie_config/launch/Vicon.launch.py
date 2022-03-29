from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    hostname = '10.196.92.136'
    buffer_size = 200
    topic_namespace = 'vicon'

    vicon_node = Node(
        package='vicon_receiver',
        executable='vicon_client',
        name='vicon_node',
        parameters=[
            {'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}
        ])

    return LaunchDescription([
        vicon_node
    ])
