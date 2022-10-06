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
            {'cf_relationship':'dron01_dron02_-0.3/-0.3/0.2, dron01_dron03_-0.4/0.0/0.1, dron02_dron01_0.3/0.3/-0.2, dron02_dron03_-0.1/0.3/-0.1, dron03_dron01_0.4/0.0/-0.1, dron03_dron02_0.1/-0.3/0.1, dron04_dron03_0.4/-0.2/0.2, dron01_khepera01_0.0/0.0/0.9'},
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

    robot01_task = Node(
        package='uned_kheperaiv_task',
        executable='shape_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera01',
        remappings=[
            ('/khepera01/khepera02/pose', '/khepera02/pose'),
            ('/khepera01/khepera03/pose', '/khepera03/pose'),
            ('/khepera01/swarm/status', '/swarm/status')],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"config_file": 'path'},
            {"agents": 'khepera02, khepera03'},
            {"agent_x": '0.4, 0.5'},
            {"agent_y": '0.0, 0.5'},
        ]
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

    robot02_task = Node(
        package='uned_kheperaiv_task',
        executable='shape_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera02',
        remappings=[
            ('/khepera02/khepera01/pose', '/khepera01/pose'),
            ('/khepera02/khepera03/pose', '/khepera03/pose'),
            ('/khepera02/swarm/status', '/swarm/status')],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"config_file": 'path'},
            {"agents": 'khepera01, khepera03'},
            {"agent_x": '-0.4, 0.1'},
            {"agent_y": ' 0.0, 0.5'},
        ]
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

    robot03_task = Node(
        package='uned_kheperaiv_task',
        executable='shape_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera03',
        remappings=[
            ('/khepera03/khepera02/pose', '/khepera02/pose'),
            ('/khepera03/khepera01/pose', '/khepera01/pose'),
            ('/khepera03/swarm/status', '/swarm/status')],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"config_file": 'path'},
            {"agents": 'khepera01, khepera02'},
            {"agent_x": '-0.5, -0.1'},
            {"agent_y": '-0.5, -0.5'},
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

    vicon_node = Node(
        package='uned_vicon_sim', 
        executable='vicon_webots',
        name='vicon_webots',
        output='screen',
        #arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"agents": 'dron01, dron02, dron03, dron04, khepera01, khepera02, khepera03'},
        ]
    )


    return LaunchDescription([
        webots,
        dron01_driver,
        dron02_driver,
        dron03_driver,
        dron04_driver,
        swarm_node,
        robot01_driver,
        robot01_task,
        robot02_driver,
        robot02_task,
        robot03_driver,
        robot03_task,
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