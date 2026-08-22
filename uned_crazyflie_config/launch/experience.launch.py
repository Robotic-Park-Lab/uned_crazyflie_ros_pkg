# Copyright 2026 Robotic Park Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Robotic Park Lab nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Uso:
    ros2 launch uned_crazyflie_config experience.launch.py config_file:=experience_swarm_teleop.yaml

Esquema del .yaml (ver resources/experience_*.yaml para ejemplos reales):

    Simulation:
      enable: true|false          # false = todos los robots son físicos
      world: RoboticPark_4cf.wbt  # solo si enable: true

    Robots:
      <id>:
        name: <id>
        type: physical|virtual|digital_twin
        driver: pid|firmware       # solo si type != physical: qué driver
                                    # de Webots usar -- pid: crazyflie.urdf
                                    # (controlador propio), firmware:
                                    # crazyflie_firmware.urdf (firmware
                                    # real de Bitcraze / gemelo digital)
        uri: radio://...           # solo si type != virtual
        config_path: <fichero.yaml en resources/, parámetros ROS del dron>

    Interface:
      rqt: {enable: true|false, file: <perspective en rqt/>}
      rviz: {enable: true|false, file: <fichero en rviz/>}

    Data_Logging:
      enable: true|false
      all: true|false             # true: 'ros2 bag record -a'
      topics: <string de topics>  # solo si all: false
      name: date|<string>         # 'date': nombre autogenerado

    Missions:                     # lista de nodos de uned_crazyflie_missions
                                    # (o cualquier otro paquete) a lanzar
      - pkg: uned_crazyflie_missions
        executable: tsp_waypoints
        name: tsp_mission          # opcional, por defecto = executable
        params: {robot_id: dron01, waypoints: [...], ...}
"""

import os
import datetime

import yaml
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_controller import WebotsController

DRIVER_URDF = {
    'pid': 'crazyflie.urdf',
    'firmware': 'crazyflie_firmware.urdf',
}


def get_ros2_nodes(context, *args):
    node_list = []
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    #-------------------#
    #     Load File     #
    #-------------------#
    config_package_dir = get_package_share_directory('uned_crazyflie_config')

    file_name = LaunchConfiguration('config_file').perform(context)
    config_path = os.path.join(config_package_dir, 'resources', file_name)
    with open(config_path, 'r') as f:
        experience = yaml.safe_load(f)

    #------------------------#
    #     Operation mode     #
    #------------------------#
    if not experience['Operation']['mode'] == 'physical':
        use_sim_time = True
        if experience['Operation']['tool'] == 'Webots':
            webots = WebotsLauncher(
                world=os.path.join(config_package_dir, 'worlds', experience['Operation']['world']),
                mode='realtime',
                ros2_supervisor=False
            )
            node_list.append(webots)
            # TO-DO: No funciona el ROS2Supervisor
            # node_list.append(Ros2SupervisorLauncher())
            
            node_list.append(launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ))
        elif experience['Operation']['tool'] == 'Gazebo':
            world_path = os.path.join(config_path, 'worlds', experience['Operation']['world'])
            gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--ros-args',
                ], output='screen'
            )
            node_list.append(gazebo)
            node_list.append(launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ))

    # -----------#
    #  Robots   #
    # -----------#
    physical_robots = []
    for robot_id, robot in experience.get('Robots', {}).items():
        if 'dron' in robot['name']:
            robot_type = robot['type']
            
            if robot_type in ('virtual', 'digital_twin'):
                urdf_name = os.path.join(config_package_dir, 'resources', 'crazyflie.urdf')
                with open(urdf_name) as f:
                    robot_desc = f.read()
                aux = robot_desc.replace("dron00", robot['name'])
                aux = aux.replace("name_id_value", robot['name'])
                aux = aux.replace("CameraAlwayOn", str(robot['camera']['alwaysOn']))
                aux = aux.replace("CameraEnable", str(robot['camera']['enable']))
                aux = aux.replace("CameraUpdateRate", str(robot['camera']['update_rate']))
                aux = aux.replace("config_file_path", config_path)

                robot_controller = WebotsController(
                    robot_name=robot['name'],
                    parameters=[
                        {'robot_description': aux,
                         'use_sim_time': use_sim_time,
                         'set_robot_state_publisher': True},
                    ],
                    respawn=True
                )
                node_list.append(robot_controller)


            if robot_type in ('physical', 'digital_twin'):
                physical_robots.append(robot['name'])

    if physical_robots:
        node_list.append(Node(
            package='uned_crazyflie_driver',
            executable='swarm_driver',
            name='swarm',
            output='screen',
            shell=True,
            emulate_tty=True,
            parameters=[
                {'config': config_path},
                {'robots': ', '.join(physical_robots)},
            ],
        ))

    #------------------------#
    #     CPU Monitoring     #
    #------------------------#
    if experience['CPU_Monitoring']['enable']:
        node_list.append(Node(
            package=experience['CPU_Monitoring']['node']['pkg'],
            executable=experience['CPU_Monitoring']['node']['executable'],
            name=experience['CPU_Monitoring']['node']['name'],
            output='screen',
            parameters=[{
                'process_name' : experience['CPU_Monitoring']['processes'],
                'process_period' : 0.5},
            ],
        ))
    
    #--------------------#
    #     Interfaces     #
    #--------------------#
    interface = experience.get('Interface', {})
    if interface['enable']:
        if interface['rqt']['enable']:
            rqt_config_path = os.path.join(config_package_dir, 'rqt', interface['rqt']['file'])
            node_list.append(Node(
                package=interface['rqt']['node']['pkg'],
                executable=interface['rqt']['node']['executable'],
                name=interface['rqt']['node']['name'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['--perspective-file', rqt_config_path],
            ))
        if interface['rviz2']['enable']:
            rviz_config_path = os.path.join(config_package_dir, 'rviz', interface['rviz2']['file'])
            node_list.append(Node(
                package=interface['rviz2']['node']['pkg'],
                executable=interface['rviz2']['node']['executable'],
                name=interface['rviz2']['node']['name'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['-d', rviz_config_path],
            ))
            node_list.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                name='RoboticPark',
                arguments=['--yaw', '3.1415', '--frame-id', 'RoboticPark/base_link', '--child-frame-id', 'map'],
            ))
        if interface['own']['enable']:
            own_config_path = os.path.join(config_package_dir, 'resources', interface['own']['file'])
            node_list.append(Node(
                package=interface['own']['node']['pkg'],
                executable=interface['own']['node']['executable'],
                name=interface['own']['node']['name'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['-d', own_config_path],
            ))

    # -----------------#
    #  Data Logging   #
    # -----------------#
    data_logging = experience.get('Data_Logging', {'enable': False})
    if data_logging.get('enable', False):
        bag_name = data_logging.get('name', 'date')
        if bag_name == 'date':
            bag_name = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M')
        if data_logging.get('all', True):
            cmd = ['ros2', 'bag', 'record', '-a', '-o', bag_name]
        else:
            cmd = ['ros2', 'bag', 'record', '-o', bag_name, data_logging.get('topics', '')]
        node_list.append(ExecuteProcess(cmd=cmd, output='screen', shell=True))


    # --------------#
    #  Missions    #
    # --------------#
    for mission in experience.get('Missions', []):
        node_list.append(Node(
            package=mission['pkg'],
            executable=mission['executable'],
            name=mission.get('name', mission['executable']),
            output='screen',
            parameters=[mission.get('params', {})],
        ))

    return node_list


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='experience_swarm_teleop.yaml',
            description='Fichero .yaml de la experiencia a lanzar, dentro de'
            ' uned_crazyflie_config/resources/'
        ),
        OpaqueFunction(function=get_ros2_nodes),
    ])
