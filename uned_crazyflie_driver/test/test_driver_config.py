# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from uned_crazyflie_driver.driver_config import resolve_driver_config

# Real per-robot config block, copied from
# uned_crazyflie_config/resources/demo_individual_waypoints_webots.yaml
# (Robots.Robot01), minus 'task' (deliberately out of scope, see
# driver_config.py's docstring).
REAL_ROBOT_CONFIG = {
    'type': 'virtual',
    'name': 'dron01',
    'control_mode': 'HighLevel',
    'positioning': 'Intern',
    'controller': {
        'type': 'pid',
        'enable': True,
        'protocol': 'Continuous',
        'period': 0.01,
        'threshold': {'type': 'Constant', 'co': 0.01, 'ai': 0.0},
    },
    'communication': {
        'type': 'Continuous',
        'threshold': {'type': 'Constant', 'co': 0.01, 'ai': 0.0},
    },
    'local_pose': {'enable': True, 'path': True, 'T': 100},
    'local_twist': {'enable': False, 'T': 50},
    'data_attitude': {'enable': False, 'T': 50},
    'data_rate': {'enable': False, 'T': 50},
    'data_motor': {'enable': False, 'T': 50},
    'data': {'enable': False, 'T': 20},
    'mars_data': {'enable': False, 'T': 50},
}


def _old_physical_derivation(config):
    """
    Reproduce el comportamiento (no defensivo) previo a esta extracción.

    El de Crazyflie_ROS2.__init__(), para comparar salida byte a byte con
    resolve_driver_config() sobre una config real.
    """
    result = {}
    result['control_mode'] = config['control_mode']
    controller_type = config['controller']['type']
    result['controller_type'] = controller_type
    if controller_type == 'ipc':
        result['controller_IPC'] = True
        result['controller_PID'] = False
    else:
        result['controller_IPC'] = False
        result['controller_PID'] = True
    result['communication'] = config['communication']['type'] == 'Continuous'
    if not result['communication']:
        result['threshold'] = config['communication']['threshold']['co']
    else:
        result['threshold'] = 0.001
    result['digital_twin'] = config['type'] == 'digital_twin'
    result['physical'] = config['type'] == 'physical'
    result['local_pose_enable'] = config['local_pose']['enable']
    result['path_enable'] = config['local_pose']['path']
    result['local_twist_enable'] = config['local_twist']['enable']
    result['data_attitude_enable'] = config['data_attitude']['enable']
    result['data_rate_enable'] = config['data_rate']['enable']
    result['data_motor_enable'] = config['data_motor']['enable']
    result['mars_data_enable'] = config['mars_data']['enable']
    result['data_enable'] = config['data']['enable']
    return result


def _old_webots_derivation(config):
    """
    Reproduce el comportamiento (defensivo) previo a esta extracción.

    El de CrazyflieWebotsDriver.init() ("key" in config con valores por
    defecto), para comparar salida byte a byte con resolve_driver_config()
    sobre una config real.
    """
    result = {}
    result['control_mode'] = config['control_mode'] if 'control_mode' in config else 'HighLevel'
    result['positioning'] = config['positioning'] if 'positioning' in config else 'Intern'
    if 'controller' in config:
        controller_type = config['controller']['type']
        result['controller_type'] = controller_type
        if controller_type == 'ipc':
            result['controller_IPC'] = True
            result['controller_PID'] = False
        else:
            result['controller_IPC'] = False
            result['controller_PID'] = True
    else:
        result['controller_type'] = 'pid'
        result['controller_IPC'] = False
        result['controller_PID'] = True
    result['physical'] = config['type'] == 'physical' if 'type' in config else False
    if 'communication' in config:
        result['communication'] = config['communication']['type'] == 'Continuous'
        if not result['communication']:
            result['threshold'] = config['communication']['threshold']['co']
        else:
            result['threshold'] = 0.001
    else:
        result['communication'] = True
        result['threshold'] = 0.001
    result['local_pose_enable'] = (
        config['local_pose']['enable'] if 'local_pose' in config else False)
    result['path_enable'] = (
        config['local_pose']['path'] if 'path' in config['local_pose'] else False)
    result['local_twist_enable'] = (
        config['local_twist']['enable'] if 'local_twist' in config else False)
    result['data_attitude_enable'] = (
        config['data_attitude']['enable'] if 'data_attitude' in config else False)
    result['data_rate_enable'] = (
        config['data_rate']['enable'] if 'data_rate' in config else False)
    result['data_motor_enable'] = (
        config['data_motor']['enable'] if 'data_motor' in config else False)
    result['mars_data_enable'] = (
        config['mars_data']['enable'] if 'mars_data' in config else False)
    result['data_enable'] = config['data']['enable'] if 'data' in config else False
    return result


def test_matches_old_physical_derivation_on_a_real_robot_config():
    resolved = resolve_driver_config(REAL_ROBOT_CONFIG)
    old = _old_physical_derivation(REAL_ROBOT_CONFIG)
    for key, value in old.items():
        assert resolved[key] == value, key


def test_matches_old_webots_derivation_on_a_real_robot_config():
    resolved = resolve_driver_config(REAL_ROBOT_CONFIG)
    old = _old_webots_derivation(REAL_ROBOT_CONFIG)
    for key, value in old.items():
        assert resolved[key] == value, key


def test_ipc_controller_sets_ipc_true_pid_false():
    config = dict(REAL_ROBOT_CONFIG, controller={'type': 'ipc'})
    resolved = resolve_driver_config(config)
    assert resolved['controller_IPC'] is True
    assert resolved['controller_PID'] is False


def test_discrete_communication_reads_explicit_threshold():
    config = dict(REAL_ROBOT_CONFIG, communication={
        'type': 'Discrete', 'threshold': {'co': 0.25}})
    resolved = resolve_driver_config(config)
    assert resolved['communication'] is False
    assert resolved['threshold'] == 0.25


def test_missing_local_pose_key_defaults_safely_instead_of_crashing():
    # Bug real detectado en CrazyflieWebotsDriver.init(): comprobaba
    # "local_pose" in config para config_local_pose, pero luego accedía a
    # config['local_pose']['path'] sin comprobar si 'local_pose' existía
    # -- KeyError si la clave faltaba del todo. resolve_driver_config()
    # arregla esto con .get(..., {}).
    config = {k: v for k, v in REAL_ROBOT_CONFIG.items() if k != 'local_pose'}
    resolved = resolve_driver_config(config)
    assert resolved['local_pose_enable'] is False
    assert resolved['path_enable'] is False


def test_empty_config_never_raises():
    resolved = resolve_driver_config({})
    assert resolved['control_mode'] == 'HighLevel'
    assert resolved['physical'] is False
    assert resolved['communication'] is True
    assert resolved['threshold'] == 0.001
