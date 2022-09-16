import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('uned_crazyflie_webots')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'crazyflie1.urdf')).read_text()
    dron02_description = pathlib.Path(os.path.join(package_dir, 'resource', 'crazyflie2.urdf')).read_text()
    
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'test.wbt')
    )

    dron01_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron01_driver',
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
        name='dron02_driver',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron02'},
        parameters=[
            {'robot_description': dron02_description,
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
        robot_state_publisher,
        dron02_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])