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
Launch único parametrizado por experiencia (tarea 8 de la Fase 2, ver
AUDIT.md en la rama doc). Sustituye a los 14+6 .launch.py de demo de
uned_crazyflie_config/uned_crazyflie_webots: para lanzar una experiencia
nueva basta con escribir un .yaml en resources/, no un .launch.py nuevo.

Adaptado de RoboticPark/roboticpark_config/launch/experience.launch.py al
alcance real de este repo (un único simulador -Webots-, sin
mars_supervisor_pkg ni roboticpark_config, con una sección Missions nueva
para lanzar nodos de uned_crazyflie_missions).

Uso:
    ros2 launch uned_crazyflie_config experience.launch.py \
        config_file:=experience_swarm_teleop.yaml

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

DRIVER_URDF = {
    'pid': 'crazyflie.urdf',
    'firmware': 'crazyflie_firmware.urdf',
}


def get_ros2_nodes(context, *args):
    node_list = []
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    config_package_dir = get_package_share_directory('uned_crazyflie_config')
    gui_package_dir = get_package_share_directory('uned_crazyflie_gui')

    file_name = LaunchConfiguration('config_file').perform(context)
    config_path = os.path.join(config_package_dir, 'resources', file_name)
    with open(config_path, 'r') as f:
        experience = yaml.safe_load(f)

    # ---------------#
    #  Simulation   #
    # ---------------#
    simulation = experience.get('Simulation', {'enable': False})
    if simulation.get('enable', False):
        webots_dir = get_package_share_directory('uned_crazyflie_webots')
        webots = WebotsLauncher(
            world=os.path.join(webots_dir, 'worlds', simulation['world']))
        node_list.append(webots)
        node_list.append(Ros2SupervisorLauncher())
        node_list.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'}],
        ))
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
        robot_type = robot['type']

        if robot_type in ('virtual', 'digital_twin'):
            urdf_name = DRIVER_URDF[robot.get('driver', 'pid')]
            webots_dir = get_package_share_directory('uned_crazyflie_webots')
            with open(os.path.join(webots_dir, 'resources', urdf_name)) as f:
                robot_description = f.read()
            node_list.append(Node(
                package='webots_ros2_driver',
                executable='driver',
                output='screen',
                name=robot['name'],
                additional_env={
                    'WEBOTS_ROBOT_NAME': robot['name'],
                    'WEBOTS_CONTROLLER_URL': controller_url_prefix() + robot['name'],
                    'WEBOTS_ROBOT_CONFIG_FILE': os.path.join(
                        config_package_dir, 'resources', robot['config_path']),
                    'WEBOTS_ROBOT_ROLE': robot_type},
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time,
                    'set_robot_state_publisher': True}],
            ))

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

    # --------------#
    #  Interface   #
    # --------------#
    interface = experience.get('Interface', {})
    rqt = interface.get('rqt', {'enable': False})
    if rqt.get('enable', False):
        # Los ficheros propios de uned_crazyflie_gui son genéricos; si no
        # se encuentran ahí, se busca en uned_crazyflie_config/rqt (los
        # ligados a una demo concreta).
        rqt_path = os.path.join(gui_package_dir, 'rqt', rqt['file'])
        if not os.path.isfile(rqt_path):
            rqt_path = os.path.join(config_package_dir, 'rqt', rqt['file'])
        node_list.append(Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='interface',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--perspective-file', rqt_path],
        ))

    rviz = interface.get('rviz', {'enable': False})
    if rviz.get('enable', False):
        rviz_path = os.path.join(gui_package_dir, 'rviz', rviz['file'])
        if not os.path.isfile(rviz_path):
            rviz_path = os.path.join(config_package_dir, 'rviz', rviz['file'])
        node_list.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': use_sim_time}],
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
