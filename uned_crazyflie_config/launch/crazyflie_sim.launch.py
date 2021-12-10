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
                {"Feedback_topic": "ground_truth/pose"},
                {"ROBOT_ID": "dron01"},
                {"DEBUG": False},
                {"ZKp": 2.0, "ZKi": 0.5, "ZKd": 0.0, "ZTd": 0.0},
                {"WKp": 25.0, "WKi": 15.0, "WKd": 0.0, "WTd": 0.0},
                {"XKp": 2.0, "XKi": 0.0, "XKd": 0.0, "XTd": 0.0},
                {"UKp": 25.0, "UKi": 1.0, "UKd": 0.0, "UTd": 0.0},
                {"YKp": 2.0, "YKi": 0.0, "YKd": 0.0, "YTd": 0.0},
                {"VKp": -25.0, "VKi": -1.0, "VKd": 0.0, "VTd": 0.0},
            ]
        )

    ])
