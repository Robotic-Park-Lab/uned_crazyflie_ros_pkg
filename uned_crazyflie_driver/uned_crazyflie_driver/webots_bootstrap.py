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
Arranque Webots y controladores PID en cascada compartidos.

Motores/sensores y los 12 `PIDController` en cascada, compartidos entre
`Crazyflie_ROS2` (cuando se instancia con `webots_node=`, en
`crazyflie_agent.py`) y `CrazyflieWebotsDriver`
(`uned_crazyflie_webots/crazyflie_driver.py`).

Extraído de un bloque idéntico byte a byte entre
`Crazyflie_ROS2.virtualCrazyflie()` y el `init()` de
`CrazyflieWebotsDriver` -- mismos nombres de dispositivo, mismas
ganancias de PID. Las ganancias son las que ya se usaban en producción
en ambos ficheros: no se han recalculado ni ajustado aquí.
"""

from uned_crazyflie_driver.pid_controller import PIDController


def init_webots_devices(robot, timestep):
    """Motores (posición infinita, en modo velocidad) y sensores del Crazyflie en Webots."""
    devices = {}
    devices['m1_motor'] = robot.getDevice("m1_motor")
    devices['m1_motor'].setPosition(float('inf'))
    devices['m1_motor'].setVelocity(-1)
    devices['m2_motor'] = robot.getDevice("m2_motor")
    devices['m2_motor'].setPosition(float('inf'))
    devices['m2_motor'].setVelocity(1)
    devices['m3_motor'] = robot.getDevice("m3_motor")
    devices['m3_motor'].setPosition(float('inf'))
    devices['m3_motor'].setVelocity(-1)
    devices['m4_motor'] = robot.getDevice("m4_motor")
    devices['m4_motor'].setPosition(float('inf'))
    devices['m4_motor'].setVelocity(1)

    devices['cam'] = robot.getDevice("camera")
    devices['cam'].disable()
    devices['imu'] = robot.getDevice("inertial unit")
    devices['imu'].enable(timestep)
    devices['gps'] = robot.getDevice("gps")
    devices['gps'].enable(timestep)
    devices['gyro'] = robot.getDevice("gyro")
    devices['gyro'].enable(timestep)
    devices['range_front'] = robot.getDevice("range_front")
    devices['range_front'].enable(timestep)
    devices['range_left'] = robot.getDevice("range_left")
    devices['range_left'].enable(timestep)
    devices['range_back'] = robot.getDevice("range_back")
    devices['range_back'].enable(timestep)
    devices['range_right'] = robot.getDevice("range_right")
    devices['range_right'].enable(timestep)
    return devices


def init_webots_cascade_controllers():
    """Los 12 PIDController en cascada: posición, velocidad, actitud y rate."""
    return {
        # Position
        'z_controller': PIDController(1.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.1, 0.01),
        'x_controller': PIDController(1.0, 0.0, 0.0, 0.0, 100, 0.5, -0.5, 0.1, 0.01),
        'y_controller': PIDController(1.0, 0.0, 0.0, 0.0, 100, 0.5, -0.5, 0.1, 0.01),
        # Velocity
        'w_controller': PIDController(25.0, 15.0, 0.0, 0.0, 100, 26.0, -16.0, 0.1, 0.01),
        'u_controller': PIDController(15.0, 0.5, 0.0, 0.0, 100, 30.0, -30.0, 0.1, 0.01),
        'v_controller': PIDController(-15.0, 0.5, 0.0, 0.0, 100, 30.0, -30.0, 0.1, 0.01),
        # Attitude
        'pitch_controller': PIDController(6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0, 0.1, 0.01),
        'roll_controller': PIDController(6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0, 0.1, 0.01),
        'yaw_controller': PIDController(18.86, 0.0, 0.0, 0.0, 100, 400.0, -400.0, 0.1, 0.01),
        # Rate
        'dpitch_controller': PIDController(250.0, 500.0, 2.5, 0.01, 100, 0.0, -0.0, 0.1, 0.01),
        'droll_controller': PIDController(250.0, 500.0, 2.5, 0.01, 100, 0.0, -0.0, 0.1, 0.01),
        'dyaw_controller': PIDController(120.0, 16.698, 0.0, 0.00, 100, 0.0, -0.0, 0.1, 0.01),
    }
