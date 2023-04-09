import os
import pathlib
import launch
import yaml
from yaml.loader import SafeLoader
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    general_package_dir = get_package_share_directory('uned_crazyflie_config')
    config_path = os.path.join(general_package_dir, 'resources', 'demo_swarm_formation_distance_four.yaml')
    rviz_config_path = os.path.join(general_package_dir, 'rviz', 'demo_swarm_formation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    dron_package_dir = get_package_share_directory('uned_crazyflie_webots')
    robot_description = pathlib.Path(os.path.join(dron_package_dir, 'resources', 'crazyflie.urdf')).read_text()
    webots = WebotsLauncher(
        world=os.path.join(dron_package_dir, 'worlds', 'RoboticPark_4cf.wbt')
    )
    ros2_supervisor = Ros2SupervisorLauncher()

    robot_node_list = []
    physical_agent_list = ''

    with open(config_path) as f:
        data = yaml.load(f, Loader=SafeLoader)
        for key, robot in data.items():
            print("###  "+robot['name']+"  ###")
            
            individual_config_path = os.path.join(general_package_dir, 'resources', robot['config_path'])


            if robot['type'] == 'virtual' or robot['type'] == 'digital_twin':
                robot_node_list.append(Node(package='webots_ros2_driver', 
                                            executable='driver', 
                                            output='screen',
                                            name=robot['name'],
                                            additional_env={'WEBOTS_ROBOT_NAME': robot['name'],
                                                            'WEBOTS_CONTROLLER_URL': controller_url_prefix() + robot['name'],
                                                            'WEBOTS_ROBOT_CONFIG_FILE': individual_config_path,
                                                            'WEBOTS_ROBOT_ROLE': robot['type']},
                                            parameters=[{   'robot_description': robot_description,
                                                            'use_sim_time': use_sim_time,
                                                            'set_robot_state_publisher': True},
                                            ]
                                        )
                )
            if robot['type'] == 'physical' or robot['type'] == 'digital_twin':
                physical_agent_list += ', '+robot['name']

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

    ros2_close = launch.actions.RegisterEventHandler(
                    event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                    )
    )

    ld = LaunchDescription()
    ld.add_action(webots)
    ld.add_action(ros2_supervisor)
    ld.add_action(robot_state_publisher)
    ld.add_action(rqt_node)
    ld.add_action(rviz_node)
    ld.add_action(vicon_node)
    for robot in robot_node_list:
        ld.add_action(robot)
        
    ld.add_action(ros2_close)

    return ld