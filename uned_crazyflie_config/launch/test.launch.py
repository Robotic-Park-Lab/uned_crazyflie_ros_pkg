from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
        package='uned_vicon_bridge',
        executable='vicon_tracker',
        name='vicon',
        output='screen',
        shell=True,
        emulate_tty=True,
        )
    dron01_controller = Node(
            package='uned_crazyflie_controllers',
            namespace='dron01',
            executable='periodic_pid_position_controller',
            name='position_controller',
            output='screen',
            shell=True,
            emulate_tty=True,
            parameters=[
                {"CONTROLLER_TYPE": "PID"},
                {"ROBOT_ID": "dron01"},
                {"CONTROLLER_MODE": "close_loop"},
                {"X_POS": "1.0"},
                {"Y_POS": "0.0"},
                {"Z_POS": "0.0"},
                {"ZKp": 2.0},
                {"ZKi": 0.5},
                {"ZKd": 0.0},
                {"ZTd": 0.0},
                {"WKp": 25.0},
                {"WKi": 15.0},
                {"WKd": 0.0},
                {"WTd": 0.0},
                {"XKp": 2.0},
                {"XKi": 0.0},
                {"XKd": 0.0},
                {"XTd": 0.0},
                {"UKp": 25.0},
                {"UKi": 1.0},
                {"UKd": 0.0},
                {"UTd": 0.0},
                {"YKp": 2.0},
                {"YKi": 0.0},
                {"YKd": 0.0},
                {"YTd": 0.0},
                {"VKp": -25.0},
                {"VKi": -1.0},
                {"VKd": 0.0},
                {"VTd": 0.0}
            ]
        )
    return LaunchDescription([
        vicon_node,
        dron01_node,
        dron01_controller
    ])
