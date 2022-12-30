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

    vicon_node = Node(
        package='uned_vicon_sim', 
        executable='vicon_webots',
        name='vicon_webots',
        output='screen',
        #arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"agents": 'dron01, dron02, dron03, dron04'},
        ]
    )


    return LaunchDescription([
        webots,
        ros2_supervisor,
        dron01_driver,
        dron02_driver,
        dron03_driver,
        dron04_driver,
        swarm_node,
        robot_state_publisher,
        rqt_node,
        vicon_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])