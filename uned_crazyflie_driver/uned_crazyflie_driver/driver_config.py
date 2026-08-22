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
Resolución de las claves de configuración por-robot comunes.

Comunes a
Crazyflie_ROS2 (crazyflie_agent.py, robot físico) y CrazyflieWebotsDriver
(webots_driver.py, robot virtual en Webots).

Antes de esta extracción, ambas clases repetían la misma lectura de
config['control_mode'], config['controller']['type'], config['communication'],
config['local_pose'], etc. en su __init__/init() -- la versión física
accedía directamente (KeyError si faltaba una clave) y la de Webots
comprobaba "key" in config con valores por defecto. Se unifica aquí con el
estilo defensivo de Webots (superconjunto seguro: mismo resultado cuando
las claves están presentes, sin excepción cuando no lo están).

Deliberadamente NO incluye 'task' (formación): el punto en que cada clase
decide arrancar la lógica de formación difiere lo bastante entre ambas
(Crazyflie_ROS2.load_formation_params() se autolimita comprobando
config['task']['enable'] en varios sitios de crazyflie_agent.py;
CrazyflieWebotsDriver usa self.task_config/self.task_onboard como puerta
previa en initialize()) como para forzarlo aquí sin más riesgo del que
merece esta pasada -- ver AUDIT.md (rama doc) para el detalle.
"""


def resolve_driver_config(config):
    resolved = {}

    resolved['control_mode'] = config.get('control_mode', 'HighLevel')
    resolved['positioning'] = config.get('positioning', 'Intern')

    controller_type = config.get('controller', {}).get('type', 'pid')
    resolved['controller_type'] = controller_type
    resolved['controller_IPC'] = controller_type == 'ipc'
    resolved['controller_PID'] = not resolved['controller_IPC']

    resolved['physical'] = config.get('type') == 'physical'
    resolved['digital_twin'] = config.get('type') == 'digital_twin'

    communication = config.get('communication', {})
    resolved['communication'] = communication.get('type', 'Continuous') == 'Continuous'
    if resolved['communication']:
        resolved['threshold'] = 0.001
    else:
        resolved['threshold'] = communication['threshold']['co']

    local_pose = config.get('local_pose', {})
    resolved['local_pose_enable'] = local_pose.get('enable', False)
    resolved['path_enable'] = local_pose.get('path', False)

    resolved['local_twist_enable'] = config.get('local_twist', {}).get('enable', False)
    resolved['data_attitude_enable'] = config.get('data_attitude', {}).get('enable', False)
    resolved['data_rate_enable'] = config.get('data_rate', {}).get('enable', False)
    resolved['data_motor_enable'] = config.get('data_motor', {}).get('enable', False)
    resolved['mars_data_enable'] = config.get('mars_data', {}).get('enable', False)
    resolved['data_enable'] = config.get('data', {}).get('enable', False)

    return resolved
