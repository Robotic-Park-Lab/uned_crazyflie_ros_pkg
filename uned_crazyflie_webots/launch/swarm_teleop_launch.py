import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    dron_package_dir = get_package_share_directory('uned_crazyflie_webots')
    dron_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'crazyflie.urdf')).read_text()
    rviz_config_path = os.path.join(dron_package_dir, 'rviz', 'test.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    webots = WebotsLauncher(
        world=os.path.join(dron_package_dir, 'worlds', 'RoboticPark_4cf.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    dron01_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron01',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron01',
                        'WEBOTS_ROBOT_NAME': 'dron01'},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron02_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron02',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron02',
                        'WEBOTS_ROBOT_NAME': 'dron02'},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron03_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron03',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron03',
                        'WEBOTS_ROBOT_NAME': 'dron03'},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron04_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron04',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron04',
                        'WEBOTS_ROBOT_NAME': 'dron04'},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
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

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]

    )

    return LaunchDescription([
        webots,
        ros2_supervisor,
        dron01_driver,
        dron02_driver,
        dron03_driver,
        dron04_driver,
        robot_state_publisher,
        rqt_node,
        rviz_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])