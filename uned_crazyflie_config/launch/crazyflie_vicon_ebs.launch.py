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
            {'cf_uri': 'radio://0/80/2M/E7E7E7E701'}
        ])
    vicon_node = Node(
        package='vicon_receiver',
        executable='vicon_client',
        name='vicon_node',
        parameters=[
            {'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}
        ])
    controller_node = Node(
        package='uned_crazyflie_controllers',
        namespace='dron01',
        executable='eventbased_pid_position_controller',
        name='position_controller',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {"ROBOT_ID": "dron01"},
            {"Feedback_topic": "pose"},
            {"DEBUG": True},
            {"ZKp": 2.0, "ZKi": 0.5, "ZKd": 0.0, "ZTd": 0.0},
            {"WKp": 25.0, "WKi": 15.0, "WKd": 0.0, "WTd": 0.0},
            {"XKp": 2.0, "XKi": 1.0, "XKd": 0.0, "XTd": 0.0},
            {"UKp": 25.0, "UKi": 1.0, "UKd": 0.0, "UTd": 0.0},
            {"YKp": 2.0, "YKi": 0.0, "YKd": 0.0, "YTd": 0.0},
            {"VKp": -25.0, "VKi": -1.0, "VKd": 0.0, "VTd": 0.0},
            {"Feedback_topic": "cf_pose"},
            {"Zco": 0.01, "Zai": 0.015},
            {"Wco": 0.01, "Wai": 0.015},
            {"Xco": 0.01, "Xai": 0.015},
            {"Uco": 0.01, "Uai": 0.015},
            {"Yco": 0.01, "Yai": 0.015},
            {"Vco": 0.01, "Vai": 0.015}
        ])
    return LaunchDescription([
        dron01_node,
        vicon_node,
        controller_node
    ])
