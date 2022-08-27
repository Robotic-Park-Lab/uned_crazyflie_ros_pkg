from launch import LaunchDescription
from launch_ros.actions import Node
import datetime
from launch.actions import ExecuteProcess

def generate_launch_description():
    hostname = '10.196.92.136'
    buffer_size = 200
    topic_namespace = 'vicon'
    e = datetime.datetime.now()

    dron01_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'cf_first_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'cf_num_uri': 1},
            {'cf_control_mode': 'HighLevel, HighLevel, HighLevel, HighLevel, HighLevel'},
            {'cf_controller_type': 'Continuous, Continuous, EventBased, Continuous, EventBased'}
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
    trayectory_node = Node(
        package='uned_crazyflie_controllers',
        executable='trajectory_controller',
        namespace='dron01',
        remappings=[
                    ('/dron01/swarm/status', '/swarm/status')],
        name='trayectory'
    )
    return LaunchDescription([
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-o', e.strftime("%Y-%m-%d-%H-%M"), ], output='screen'),
        dron01_node,
        trayectory_node,
        vicon_node,
        rqt_node
    ])
