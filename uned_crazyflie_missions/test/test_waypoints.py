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

from uned_crazyflie_missions.waypoints import load_points, order_for_shape
from uned_crazyflie_missions.tsp import path_length


def test_load_points_preserves_yaml_order_and_defaults_t():
    points_config = {
        'P0': {'x': 0.0, 'y': 1.0, 'z': 1.0, 't': 5.0},
        'P1': {'x': 0.866, 'y': 0.5, 'z': 1.0},  # no 't' -> defaults to 0.0
    }
    points = load_points(points_config)
    assert points == [(0.0, 1.0, 1.0, 5.0), (0.866, 0.5, 1.0, 0.0)]


def test_order_for_shape_polygon_is_declaration_order():
    points = [(0, 0, 1, 0), (10, 0, 1, 0), (10, 10, 1, 0), (0, 10, 1, 0)]
    assert order_for_shape('polygon', points) == [0, 1, 2, 3]


def test_order_for_shape_tsp_finds_the_optimal_square_order():
    # Same square used in uned_crazyflie_missions' own TSP tests: optimal
    # order is the perimeter (30), not the diagonal-crossing order.
    points = [(0, 0, 1, 0), (10, 0, 1, 0), (0, 10, 1, 0), (10, 10, 1, 0)]
    order = order_for_shape('tsp', points)
    assert sorted(order) == [0, 1, 2, 3]
    xyz = [(x, y, z) for x, y, z, _t in points]
    assert path_length(order, xyz) == 30.0


def test_order_for_shape_tsp_handles_a_single_point():
    assert order_for_shape('tsp', [(0, 0, 1, 0)]) == [0]
