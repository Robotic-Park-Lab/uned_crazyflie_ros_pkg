import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable
from launch.actions import SetEnvironmentVariable
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('uned_crazyflie_webots')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'crazyflie.urdf')).read_text()
    
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'test.wbt')
    )

    dron01_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron01',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron01'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': True,
             'set_robot_state_publisher': True},
        ]
    )

    dron02_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron02',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron02'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': True,
             'set_robot_state_publisher': True},
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    return LaunchDescription([
        webots,
        dron01_driver,
        dron02_driver,
        robot_state_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])