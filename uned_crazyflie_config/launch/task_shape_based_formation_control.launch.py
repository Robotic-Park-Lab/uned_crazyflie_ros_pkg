from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hostname = '10.196.92.136'
    buffer_size = 200
    topic_namespace = 'vicon'

    dron01_node = Node(
        package='uned_crazyflie_task',
        executable='shape_based_formation_control',
        name='FormationControl',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'cf_first_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'cf_num_uri': 3},
            {'cf_control_mode': 'HighLevel, HighLevel, HighLevel'},
            {'cf_controller_type': 'Continuous, Continuous, Continuous'},
            {'cf_role': 'consensus, consensus, consensus'},
            {'cf_relationship':'dron01_khepera01_-0.2/-0.2/0.7, dron01_dron02_-0.3/-0.3/0.2, dron01_dron03_-0.4/0.0/0.1, dron02_dron01_0.3/0.3/-0.2, dron02_dron03_-0.1/0.3/-0.1, dron02_khepera02_0.6/0.6/0.5, dron03_dron01_0.4/0.0/-0.1, dron03_dron02_0.1/-0.3/0.1'},
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
        dron01_node,
        vicon_node
        # rqt_node

    ])
