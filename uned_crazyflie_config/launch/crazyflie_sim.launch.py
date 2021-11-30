from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uned_crazyflie_controllers',
            namespace='dron01',
            executable='periodic_pid_position_controller',
            name='position_controller',
            parameters=[
                {"CONTROLLER_TYPE": "PID"},
                {"ROBOT_ID": "dron01"},
                {"CONTROLLER_MODE": "close_loop"},
                {"X_POS": "1.0"},
                {"Y_POS": "0.0"},
                {"Z_POS": "0.0"},
                {"ZKp": 15.0},
                {"ZKi": 3.5241},
                {"ZKd": 0.0},
                {"ZTd": 0.0},
                {"WKp": 25.0},
                {"WKi": 15.0},
                {"WKd": 0.0},
                {"WTd": 0.0},
                {"XKp": 25.0},
                {"XKi": 1.0},
                {"XKd": 0.0},
                {"XTd": 0.0},
                {"UKp": 25.0},
                {"UKi": 1.0},
                {"UKd": 0.0},
                {"UTd": 0.0},
                {"YKp": 25.0},
                {"YKi": 1.0},
                {"YKd": 0.0},
                {"YTd": 0.0},
                {"VKp": -25.0},
                {"VKi": -1.0},
                {"VKd": 0.0},
                {"VTd": 0.0}
            ]
        )

    ])
