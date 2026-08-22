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

from math import pi

from builtin_interfaces.msg import Time as TimeMsg

from uned_crazyflie_driver.sensors import agent_removal_marker, build_laserscan


def test_build_laserscan_orders_ranges_back_left_front_right_back():
    stamp = TimeMsg(sec=1, nanosec=0)
    msg = build_laserscan(1.0, 2.0, 3.0, 0.5, stamp, 'dron01')
    # front=1.0, back=2.0, left=3.0, right=0.5
    assert list(msg.ranges) == [2.0, 3.0, 1.0, 0.5, 2.0]
    assert msg.header.frame_id == 'dron01'
    assert msg.header.stamp == stamp


def test_build_laserscan_saturates_above_max_range_to_infinity():
    msg = build_laserscan(10.0, 0.5, 0.5, 0.5, TimeMsg(), 'dron01')
    assert msg.ranges[2] == float('inf')  # front
    assert msg.ranges[0] == 0.5  # back, untouched


def test_build_laserscan_angle_fields_match_original_constants():
    msg = build_laserscan(0.5, 0.5, 0.5, 0.5, TimeMsg(), 'dron01')
    assert msg.angle_min == 0.5 * 2 * pi
    assert msg.angle_max == -0.5 * 2 * pi
    assert msg.angle_increment == -1.0 * pi / 2
    assert msg.range_min == 0.1
    assert msg.range_max == 3.49


def test_agent_removal_marker_is_a_degenerate_line_list_with_no_points():
    stamp = TimeMsg(sec=5, nanosec=0)
    marker = agent_removal_marker(stamp)
    assert marker.header.frame_id == 'map'
    assert marker.header.stamp == stamp
    assert marker.type == 5  # LINE_LIST
    assert marker.action == 0  # ADD (with no points => effectively clears it)
    assert list(marker.points) == []
    assert marker.scale.x == 0.01 and marker.scale.y == 0.01 and marker.scale.z == 0.01
