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
    config_path = os.path.join(general_package_dir, 'resources', 'demo_swarm_teleop_intern.yaml')
    rviz_config_path = os.path.join(general_package_dir, 'rviz', 'demo_swarm_teleop.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    dron_package_dir = get_package_share_directory('uned_crazyflie_webots')
    robot_description = pathlib.Path(os.path.join(dron_package_dir, 'resources', 'crazyflie.urdf')).read_text()
    webots = WebotsLauncher(
        world=os.path.join(dron_package_dir, 'worlds', 'RoboticPark_1cf.wbt')
    )
    ros2_supervisor = Ros2SupervisorLauncher()

    individual_config_path = os.path.join(general_package_dir, 'resources', 'crazyflie_intern_default.yaml')

    dron01 = Node(
        package='webots_ros2_driver', 
        executable='driver', 
        output='screen',
        name='dron01',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron01',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron01',
                        'WEBOTS_ROBOT_CONFIG_FILE': individual_config_path,
                        'WEBOTS_ROBOT_ROLE': 'virtual'},
        parameters=[{   'robot_description': robot_description,
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
            {"agents": 'dron01'},
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
    ld.add_action(dron01)
        
    ld.add_action(ros2_close)

    return ld