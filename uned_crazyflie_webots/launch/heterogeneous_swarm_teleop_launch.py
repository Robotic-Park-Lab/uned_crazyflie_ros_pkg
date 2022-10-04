import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    dron_package_dir = get_package_share_directory('uned_crazyflie_webots')
    robot_package_dir = get_package_share_directory('uned_kheperaiv_webots')
    dron_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'crazyflie.urdf')).read_text()
    robot_description = pathlib.Path(os.path.join(robot_package_dir, 'resource', 'kheperaiv.urdf')).read_text()
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    webots = WebotsLauncher(
        world=os.path.join(dron_package_dir, 'worlds', 'apartment_4cf_3kh.wbt')
    )

    dron01_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron01',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron01'},
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
        additional_env={'WEBOTS_ROBOT_NAME': 'dron02'},
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
        additional_env={'WEBOTS_ROBOT_NAME': 'dron03'},
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
        additional_env={'WEBOTS_ROBOT_NAME': 'dron04'},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    robot01_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='khepera01',
        additional_env={'WEBOTS_ROBOT_NAME': 'khepera01'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
    )

    robot02_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='khepera02',
        additional_env={'WEBOTS_ROBOT_NAME': 'khepera02'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
    )

    robot03_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='khepera03',
        additional_env={'WEBOTS_ROBOT_NAME': 'khepera03'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
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

    return LaunchDescription([
        webots,
        dron01_driver,
        dron02_driver,
        dron03_driver,
        dron04_driver,
        robot01_driver,
        robot02_driver,
        robot03_driver,
        robot_state_publisher,
        rqt_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])