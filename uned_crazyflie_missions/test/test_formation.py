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

from uned_crazyflie_missions.formation import compute_correction


def test_compute_correction_with_no_neighbours_is_zero():
    assert compute_correction((0.0, 0.0, 1.0), []) == (0.0, 0.0, 0.0)


def test_compute_correction_zero_when_already_at_desired_offset():
    # Neighbour is at (1, 0, 1); we want to be 1m to its "left" (dx=-1) ->
    # our desired position is (0, 0, 1), which is exactly where we are.
    own = (0.0, 0.0, 1.0)
    neighbours = [(-1.0, 0.0, 0.0, 1.0, 0.0, 1.0)]
    assert compute_correction(own, neighbours) == (0.0, 0.0, 0.0)


def test_compute_correction_pulls_toward_the_desired_offset():
    # Same desired offset as above, but we're too far from the neighbour
    # (2m instead of 1m) -- correction should pull us +1m closer in x.
    own = (0.0, 0.0, 1.0)
    neighbours = [(-1.0, 0.0, 0.0, 2.0, 0.0, 1.0)]
    dx, dy, dz = compute_correction(own, neighbours)
    assert dx == 1.0
    assert dy == 0.0
    assert dz == 0.0


def test_compute_correction_is_the_mean_over_neighbours():
    own = (0.0, 0.0, 1.0)
    # Neighbour A wants a +2 correction, neighbour B wants a -2 correction
    # -> mean should cancel out to 0.
    neighbours = [
        (0.0, 0.0, 0.0, 2.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, -2.0, 0.0, 1.0),
    ]
    dx, _dy, _dz = compute_correction(own, neighbours)
    assert dx == 0.0
