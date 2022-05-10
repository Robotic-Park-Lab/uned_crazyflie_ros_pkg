from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uned_crazyflie_controllers',
            namespace='dron01',
            executable='fuzzy_pid_position_controller',
            name='position_controller',
            parameters=[
                {"Feedback_pose_topic": "ground_truth/pose"},
                {"Feedback_twist_topic": "odometry/twist/twist"},
                {"ROBOT_ID": "dron01"},
                {"DEBUG": True},
                {"ZKp": 2.0, "ZKi": 0.5, "ZKd": 0.0, "ZTd": 0.0},
                {"WKp": 25.0, "WKi": 15.0, "WKd": 0.0, "WTd": 0.0},
                {"XKe": 0.02, "XKd": 0.035, "XK0": 5.0, "XK1": 0.25},
                {"YKe": 0.02, "YKd": 0.035, "YK0": 5.0, "YK1": 0.25},
            ]
        )

    ])
