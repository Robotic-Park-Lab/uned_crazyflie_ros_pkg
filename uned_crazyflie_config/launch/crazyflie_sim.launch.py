from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    dron01_pos_node = Node(
        package='uned_crazyflie_controllers',
        namespace='dron01',
        executable='periodic_pid_position_controller',
        name='position_controller',
        parameters=[
            {"Feedback_pose_topic": "ground_truth/pose"},
            {"Feedback_twist_topic": "odometry/twist/twist"},
            {"ROBOT_ID": "dron01"},
            {"DEBUG": False},
            {"ZKp": 2.0, "ZKi": 0.5, "ZKd": 0.0, "ZTd": 0.0},
            {"WKp": 25.0, "WKi": 15.0, "WKd": 0.0, "WTd": 0.0},
            {"XKp": 2.0, "XKi": 0.0, "XKd": 0.0, "XTd": 0.0},
            {"UKp": 15.0, "UKi": 0.0, "UKd": 0.0, "UTd": 0.0},
            {"YKp": 2.0, "YKi": 0.0, "YKd": 0.0, "YTd": 0.0},
            {"VKp": -15.0, "VKi": 0.0, "VKd": 0.0, "VTd": 0.0},
        ]
    )
    vicon_node = Node(
        package='vicon_gazebo',
        executable='vicon_gazebo',
        namespace='dron01',
        name='vicon'
    )
    trayectory_node = Node(
        package='uned_crazyflie_controllers',
        executable='trajectory_controller',
        namespace='dron01',
        name='trayectory'
    )
    return LaunchDescription([
        dron01_pos_node,
        vicon_node,
        trayectory_node

    ])
