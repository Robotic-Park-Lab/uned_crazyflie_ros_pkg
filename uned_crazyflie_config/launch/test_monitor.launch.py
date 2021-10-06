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
                {"Z_POS": "0.0"},
                {"Zq1":   "915017.5"},
                {"Zq2": "-1814982.5"},
                {"Zq3":   "900000.0"}
            ]
        ),
        Node(
            package='uned_crazyflie_driver',
            namespace='dron01',
            executable='crazyflie_driver',
            name='cf_driver'
        ),
        Node(
            package='uned_vicon_bridge',
            executable='vicon_tracker',
            name='vicon_tracker'
        )
    ])
