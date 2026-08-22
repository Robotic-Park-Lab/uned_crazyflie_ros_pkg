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
when experience.launch.py was written (task 8) and re-verified against
Francisco's own reference experience files (2026-08-22): running
get_ros2_nodes() with a real LaunchContext against real experience files
and checking it produces the expected actions -- not just that the file
parses, but that each section of the schema (Operation/Robots/
Interface/Data_Logging/Missions) actually contributes what it should.

Runs against the installed share/ directory (not the source tree), so it
also catches packaging mistakes (a resource not installed, etc).
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


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


def test_teleop_webots_experience_launches_webots_and_interface():
    actions, nodes = _get_nodes('demo_individual_teleop_webots.yaml')

    assert any(isinstance(a, WebotsLauncher) for a in actions), \
        'Operation.mode: virtual + tool: Webots must add a WebotsLauncher'
    assert any(isinstance(a, RegisterEventHandler) for a in actions), \
        'must register a shutdown handler tied to Webots exiting'
    assert any(isinstance(a, WebotsController) for a in actions), \
        'the 1 virtual robot in the yaml must get a WebotsController'

    packages = [(n.node_package, n.node_executable) for n in nodes]
    assert ('rviz2', 'rviz2') in packages, 'Interface.rviz2.enable: true must add rviz2'
    assert ('rqt_gui', 'rqt_gui') not in packages, 'Interface.rqt.enable: false must not add it'
    assert ('measure_process_ros2_pkg', 'measure_process') in packages, \
        'CPU_Monitoring.enable: true must add its node'
    assert not any(pkg == 'uned_crazyflie_driver' for pkg, _ in packages), \
        'no robot is physical/digital_twin, so swarm_driver must not be launched'


def test_teleop_vicon_experience_adds_vicon_receiver_and_no_webots_launcher():
    actions, nodes = _get_nodes('demo_individual_teleop_vicon.yaml')

    # This experience doesn't launch Webots itself (no WebotsLauncher), but
    # its one robot is still 'type: virtual', so it still gets a
    # WebotsController to drive an already-running external Webots instance.
    assert not any(isinstance(a, WebotsLauncher) for a in actions)
    assert any(isinstance(a, WebotsController) for a in actions)

    packages = [(n.node_package, n.node_executable) for n in nodes]
    assert ('vicon_receiver', 'vicon_client') in packages, \
        'this experience must launch the Vicon receiver'


def test_waypoints_webots_experience_launches_missions():
    actions, nodes = _get_nodes('demo_individual_waypoints_webots.yaml')

    packages = [(n.node_package, n.node_executable) for n in nodes]
    assert ('uned_crazyflie_missions', 'sequencer') in packages, \
        'the Missions section must launch the configured sequencer node'
    assert ('uned_crazyflie_missions', 'waypoints') in packages, \
        'the Missions section must launch the configured waypoints node'


def test_experience_yaml_resources_are_installed():
    share_dir = get_package_share_directory('uned_crazyflie_config')
    for name in ('demo_individual_teleop_webots.yaml', 'demo_individual_teleop_vicon.yaml',
                 'demo_individual_waypoints_webots.yaml',
                 'demo_individual_waypoints.yaml', 'demo_individual_waypoints_topics.yaml',
                 'crazyflie.urdf'):
        path = os.path.join(share_dir, 'resources', name)
        assert os.path.isfile(path), '%s must be installed under resources/' % name


def test_interface_files_referenced_by_examples_resolve_on_disk():
    gui_dir = get_package_share_directory('uned_crazyflie_gui')
    assert os.path.isfile(os.path.join(gui_dir, 'rqt', 'crazyflie.perspective'))
    assert os.path.isfile(os.path.join(gui_dir, 'rviz', 'crazyflie.rviz'))
    config_dir = get_package_share_directory('uned_crazyflie_config')
    assert os.path.isfile(os.path.join(config_dir, 'rviz', 'demo_individual.rviz'))
