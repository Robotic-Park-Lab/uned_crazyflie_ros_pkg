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
Formalizes, as a real pytest test, the manual verification already done
when experience.launch.py was written (task 8): running get_ros2_nodes()
with a real LaunchContext against the two example experience files and
checking it produces the expected actions -- not just that the file
parses, but that each section of the schema (Simulation/Robots/
Interface/Data_Logging/Missions) actually contributes what it should.

Runs against the installed share/ directory (not the source tree), so it
also catches packaging mistakes (a resource not installed, etc).
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


def _load_experience_launch():
    share_dir = get_package_share_directory('uned_crazyflie_config')
    launch_path = os.path.join(share_dir, 'launch', 'experience.launch.py')
    spec = importlib.util.spec_from_file_location('experience_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _get_nodes(config_file):
    module = _load_experience_launch()
    context = LaunchContext()
    context.launch_configurations['config_file'] = config_file
    actions = module.get_ros2_nodes(context)
    nodes = [a for a in actions if isinstance(a, Node)]
    return actions, nodes


def _bag_record_processes(actions):
    # WebotsLauncher, Ros2SupervisorLauncher and Node are all themselves
    # ExecuteProcess subclasses (they launch external processes too), so
    # isinstance(a, ExecuteProcess) matches far more than just the 'ros2
    # bag record' action added by Data_Logging -- match the exact type
    # instead, since the launch file constructs a plain ExecuteProcess(...)
    # for the bag recording and nothing else in this launch does.
    return [a for a in actions if type(a) is ExecuteProcess]


def _resolved(substitution_or_str, context):
    if isinstance(substitution_or_str, str):
        return substitution_or_str
    return substitution_or_str.perform(context)


def test_swarm_teleop_experience_launches_two_virtual_robots_and_interface():
    actions, nodes = _get_nodes('experience_swarm_teleop.yaml')

    assert any(isinstance(a, WebotsLauncher) for a in actions), \
        'Simulation.enable: true must add a WebotsLauncher'
    assert any(isinstance(a, RegisterEventHandler) for a in actions), \
        'must register a shutdown handler tied to Webots exiting'

    packages = [(n.node_package, n.node_executable) for n in nodes]
    assert packages.count(('webots_ros2_driver', 'driver')) == 2, \
        'the 2 virtual robots in the yaml must each get a webots_ros2_driver/driver node'
    assert ('rqt_gui', 'rqt_gui') in packages, 'Interface.rqt.enable: true must add rqt_gui'
    assert ('rviz2', 'rviz2') in packages, 'Interface.rviz.enable: true must add rviz2'
    assert not any(pkg == 'uned_crazyflie_driver' for pkg, _ in packages), \
        'no robot is physical/digital_twin, so swarm_driver must not be launched'
    assert not _bag_record_processes(actions), \
        'Data_Logging.enable: false must not add a ros2 bag record process'


def test_tsp_digital_twin_experience_launches_swarm_driver_and_mission():
    actions, nodes = _get_nodes('experience_tsp_digital_twin.yaml')

    packages = [(n.node_package, n.node_executable) for n in nodes]
    assert ('webots_ros2_driver', 'driver') in packages, \
        'a digital_twin robot must still get its Webots visual twin'
    assert ('uned_crazyflie_driver', 'swarm_driver') in packages, \
        'a digital_twin robot must also be flown for real via swarm_driver'
    assert ('uned_crazyflie_missions', 'tsp_waypoints') in packages, \
        'the Missions section must launch the configured mission node'
    assert ('rqt_gui', 'rqt_gui') not in packages, \
        'Interface.rqt.enable: false must not add rqt_gui'
    assert _bag_record_processes(actions), \
        'Data_Logging.enable: true must add a ros2 bag record process'


def test_experience_yaml_resources_are_installed():
    share_dir = get_package_share_directory('uned_crazyflie_config')
    for name in ('experience_swarm_teleop.yaml', 'experience_tsp_digital_twin.yaml'):
        path = os.path.join(share_dir, 'resources', name)
        assert os.path.isfile(path), '%s must be installed under resources/' % name


def test_interface_files_referenced_by_examples_resolve_on_disk():
    gui_dir = get_package_share_directory('uned_crazyflie_gui')
    assert os.path.isfile(os.path.join(gui_dir, 'rqt', 'crazyflie.perspective'))
    assert os.path.isfile(os.path.join(gui_dir, 'rviz', 'crazyflie.rviz'))
