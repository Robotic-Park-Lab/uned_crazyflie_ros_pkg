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
Aplicación de parámetros PID (mensaje Pidcontroller) a un Crazyflie real.

Extraído de un bloque de ~150 líneas duplicado 4 veces (idéntico o casi
idéntico) en `uned_crazyflie_task/leader_follower.py`,
`uned_crazyflie_task/shape_based_formation_control.py`,
`uned_crazyflie_driver/crazyflie_agent.py` y
`uned_crazyflie_driver/swarm_driver.py`. Las claves de parámetro de
firmware (nombres de grupo, sufijos `_kp`/`_ki`/`_kd`, y el caso especial
`xyVelMax` compartido entre los ejes x/y) se preservan tal cual estaban en
las 4 copias -- no son typos, son los nombres reales que usa el firmware
del Crazyflie.
"""

# Ejes de posición: mismo Kp/Ki/Kd(/Co/Ai) + un límite de velocidad
# (VelMax) cuya clave de parámetro varía por eje. x e y comparten el mismo
# parámetro real de firmware ('<group>.xyVelMax'), construido de forma
# distinta en cada eje en el código original; se preserva tal cual.
_POS_AXES = {
    'x': lambda id_: id_ + 'yVelMax',
    'y': lambda id_: 'x' + id_ + 'VelMax',
    'z': lambda id_: id_ + 'VelMax',
}
# Ejes de velocidad: solo Kp/Ki/Kd(/Co/Ai), sin límite de velocidad propio.
_VEL_AXES = ('vx', 'vy', 'vz')
# Ejes de actitud: grupo fijo 'pid_attitude', sufijo '_kp'/'_ki'/'_kd',
# igual en modo continuo y event-based.
_ATTITUDE_AXES = ('roll', 'pitch', 'yaw')
# Ejes de velocidad angular (rate): el id del mensaje empieza por 'd'
# ('droll', 'dpitch', 'dyaw') pero el parámetro real de firmware usa el
# nombre del eje sin la 'd' ('roll_kp', 'pitch_kp', 'yaw_kp').
_RATE_AXES = {'droll': 'roll', 'dpitch': 'pitch', 'dyaw': 'yaw'}


def _set_pos_or_vel_axis(set_value, group, msg, velmax_key):
    set_value(group + '.' + msg.id + 'Kp', msg.kp)
    set_value(group + '.' + msg.id + 'Ki', msg.ki)
    set_value(group + '.' + msg.id + 'Kd', msg.kd)
    if velmax_key is not None:
        set_value(group + '.' + velmax_key, msg.upperlimit)


def _set_pos_or_vel_axis_eventbased(set_value, group, msg, velmax_key):
    set_value(group + '.' + msg.id + 'Kp', msg.kp)
    set_value(group + '.' + msg.id + 'Ki', msg.ki)
    set_value(group + '.' + msg.id + 'Kd', msg.kd)
    set_value(group + '.' + msg.id + 'Co', msg.co)
    set_value(group + '.' + msg.id + 'Ai', msg.ai)
    if velmax_key is not None:
        set_value(group + '.' + velmax_key, msg.upperlimit)


def apply_controller_params(set_value, log_info, msg, event_based):
    """
    Aplica un mensaje Pidcontroller a los parámetros de firmware del CF.

    - set_value: callable(param_path: str, value) -> None
      (normalmente scf.cf.param.set_value o cf.param.set_value).
    - log_info: callable(str) -> None (normalmente logger.info), se llama
      al final con el mismo resumen que tenían las 4 copias originales.
    - msg: mensaje uned_crazyflie_config/Pidcontroller.
    - event_based: True para el grupo *EventBased* (posEbCtlPid,
      velEbCtlPid, con Co/Ai), False para el grupo *Continuous*
      (posCtlPid, velCtlPid, sin Co/Ai). Los grupos de actitud/rate
      (pid_attitude/pid_rate) son los mismos en ambos casos.
    """
    if msg.id in _POS_AXES:
        velmax_key = _POS_AXES[msg.id](msg.id)
        if event_based:
            _set_pos_or_vel_axis_eventbased(
                set_value, 'posEbCtlPid', msg, velmax_key)
        else:
            _set_pos_or_vel_axis(set_value, 'posCtlPid', msg, velmax_key)
    elif msg.id in _VEL_AXES:
        if event_based:
            _set_pos_or_vel_axis_eventbased(set_value, 'velEbCtlPid', msg, None)
        else:
            _set_pos_or_vel_axis(set_value, 'velCtlPid', msg, None)
    elif msg.id in _ATTITUDE_AXES:
        set_value('pid_attitude.' + msg.id + '_kp', msg.kp)
        set_value('pid_attitude.' + msg.id + '_ki', msg.ki)
        set_value('pid_attitude.' + msg.id + '_kd', msg.kd)
    elif msg.id in _RATE_AXES:
        base = _RATE_AXES[msg.id]
        set_value('pid_rate.' + base + '_kp', msg.kp)
        set_value('pid_rate.' + base + '_ki', msg.ki)
        set_value('pid_rate.' + base + '_kd', msg.kd)

    log_info(
        'Kp: %0.2f \t Ki: %0.2f \t Kd: %0.2f \t N: %0.2f \t UL: %0.2f \t LL: %0.2f' %
        (msg.kp, msg.ki, msg.kd, msg.nd, msg.upperlimit, msg.lowerlimit))
