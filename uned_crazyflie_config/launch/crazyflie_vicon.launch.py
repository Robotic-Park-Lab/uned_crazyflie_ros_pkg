from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    hostname = '10.196.88.156'
    buffer_size = 200
    topic_namespace = 'vicon'

    dron01_node = Node(
        package='uned_crazyflie_driver',
        namespace='dron01',
        executable='crazyflie_driver',
        name='crazyflie',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'cf_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'cf_control_mode': 'HighLevel'}
        ])
    vicon_node = Node(
        package='vicon_receiver',
        executable='vicon_client',
        name='vicon_node',
        parameters=[
            {'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}
        ])
    return LaunchDescription([
        dron01_node,
        vicon_node
    ])
