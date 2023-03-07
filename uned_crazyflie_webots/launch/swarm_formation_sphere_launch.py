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
    config_package_dir = get_package_share_directory('uned_crazyflie_config')
    dron01_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'MRS_sphere_dron01.urdf')).read_text()
    dron02_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'MRS_sphere_dron02.urdf')).read_text()
    dron03_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'MRS_sphere_dron03.urdf')).read_text()
    dron04_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'MRS_sphere_dron04.urdf')).read_text()
    dron05_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'MRS_sphere_dron05.urdf')).read_text()
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'sphere.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    webots = WebotsLauncher(
        world=os.path.join(dron_package_dir, 'worlds', 'RoboticPark_MRS_sphere.wbt')
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
            {'robot_description': dron01_description,
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
            {'robot_description': dron02_description,
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
            {'robot_description': dron03_description,
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
            {'robot_description': dron04_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron05_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron05',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron05',
                        'WEBOTS_ROBOT_NAME': 'dron05'},
        parameters=[
            {'robot_description': dron05_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    vicon_node = Node(
        package='uned_vicon_sim', 
        executable='vicon_webots',
        name='vicon_webots',
        output='screen',
        #arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"agents": 'dron01, dron02, dron03, dron04, dron05'},
        ]
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
        dron05_driver,
        rqt_node,
        vicon_node,
        rviz_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])