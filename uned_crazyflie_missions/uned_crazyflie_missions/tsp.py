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
Heurístico simple para el TSP sobre un conjunto de waypoints 3D.

Vecino más cercano + mejora 2-opt acotada. No es un solver óptimo -- para
el número de waypoints de una misión real (decenas, no miles) es más que
suficiente y muy rápido.

Funciones puras, sin dependencia de ROS, para poder testearlas sin
necesidad de rclpy.
"""

from math import sqrt


def _distance(a, b):
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def path_length(order, points):
    """Longitud total del recorrido que visita points[order[0]], points[order[1]], ..."""
    return sum(
        _distance(points[order[i]], points[order[i + 1]])
        for i in range(len(order) - 1)
    )


def nearest_neighbor(points, start=0):
    """Orden de visita por el heurístico del vecino más cercano, empezando en `start`."""
    n = len(points)
    unvisited = set(range(n))
    unvisited.discard(start)
    order = [start]
    current = start
    while unvisited:
        nxt = min(unvisited, key=lambda i: _distance(points[current], points[i]))
        order.append(nxt)
        unvisited.discard(nxt)
        current = nxt
    return order


def two_opt(order, points, max_passes=50):
    """Mejora local 2-opt: invierte tramos del recorrido mientras acorte la distancia total."""
    order = list(order)
    n = len(order)
    if n < 4:
        return order

    improved = True
    passes = 0
    while improved and passes < max_passes:
        improved = False
        passes += 1
        for i in range(1, n - 2):
            for j in range(i + 1, n - 1):
                a, b = points[order[i - 1]], points[order[i]]
                c, d = points[order[j]], points[order[j + 1]]
                before = _distance(a, b) + _distance(c, d)
                after = _distance(a, c) + _distance(b, d)
                if after < before - 1e-9:
                    order[i:j + 1] = reversed(order[i:j + 1])
                    improved = True
    return order


def solve_tsp(points, start=0):
    """
    Vecino más cercano desde `start`, seguido de un refinado 2-opt.

    Devuelve el orden de índices a visitar.
    """
    order = nearest_neighbor(points, start=start)
    return two_opt(order, points)
