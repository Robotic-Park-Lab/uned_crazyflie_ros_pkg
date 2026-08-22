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
Piezas puntuales antes duplicadas entre los dos drivers.

Compartidas entre Crazyflie_ROS2 (crazyflie_agent.py) y
CrazyflieWebotsDriver (webots_driver.py) construían con código idéntico:

- build_laserscan(): antes duplicado byte a byte como
  Crazyflie_ROS2.publish_laserscan() / CrazyflieWebotsDriver.publish_laserscan_data().
- agent_removal_marker(): antes duplicado (4 veces en total entre las dos
  clases: Crazyflie_ROS2.disconnected()/remove_agent(),
  CrazyflieWebotsDriver.remove_agent()/_disconnected()) para construir el
  Marker que borra la línea RViz de un vecino al desconectarlo.

Ambas son funciones puras -- no dependen de rclpy ni de ningún estado de
la clase -- para poder testearlas sin un grafo ROS.
"""

from math import pi

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

MAX_RANGE = 3.49


def build_laserscan(front_range, back_range, left_range, right_range, stamp, frame_id):
    """
    Construye un LaserScan a partir de las 4 lecturas de distancia.

    Satura a infinito por encima de MAX_RANGE. Rangos ya en metros.
    """
    def clamp(value):
        return float('inf') if value > MAX_RANGE else value

    msg = LaserScan()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.range_min = 0.1
    msg.range_max = MAX_RANGE
    msg.ranges = [
        clamp(back_range), clamp(left_range), clamp(front_range), clamp(right_range),
        clamp(back_range),
    ]
    msg.angle_min = 0.5 * 2 * pi
    msg.angle_max = -0.5 * 2 * pi
    msg.angle_increment = -1.0 * pi / 2
    return msg


def agent_removal_marker(stamp):
    """Marker degenerado (sin puntos) usado para borrar la línea RViz de un vecino."""
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = stamp
    marker.id = 1
    marker.type = 5
    marker.action = 0
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    return marker
