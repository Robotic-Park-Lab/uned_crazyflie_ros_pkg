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
        world=os.path.join(dron_package_dir, 'worlds', 'test.wbt')
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

    swarm_node = Node(
        package='uned_crazyflie_task',
        executable='formation_control_webots',
        name='CFFormationControl',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'cf_first_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'cf_num_uri': 4},
            {'cf_control_mode': 'HighLevel, HighLevel, HighLevel, HighLevel'},
            {'cf_controller_type': 'Continuous, Continuous, Continuous, Continuous'},
            {'cf_role': 'consensus, consensus, consensus, consensus'},
            {'cf_relationship':'dron01_dron02_-0.3/-0.3/0.2, dron01_dron03_-0.4/0.0/0.1, dron02_dron01_0.3/0.3/-0.2, dron02_dron03_-0.1/0.3/-0.1, dron03_dron01_0.4/0.0/-0.1, dron03_dron02_0.1/-0.3/0.1, dron04_dron03_0.4/-0.2/0.2'},
        ])

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

    return LaunchDescription([
        webots,
        dron01_driver,
        dron02_driver,
        dron03_driver,
        dron04_driver,
        swarm_node,
        robot01_driver,
        robot02_driver,
        robot03_driver,
        robot_state_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])