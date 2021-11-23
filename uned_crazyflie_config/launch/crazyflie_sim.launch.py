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
                {"ROBOT_ID": "dron_test"},
                {"CONTROLLER_MODE": "close_loop"},
                {"X_POS": "1.0"},
                {"Y_POS": "0.0"},
                {"Z_POS": "0.0"}
            ]
        )
    ])
