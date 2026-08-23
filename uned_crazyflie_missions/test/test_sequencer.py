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

from uned_crazyflie_missions.sequencer import sorted_cmd_keys, trigger_satisfied


def test_sorted_cmd_keys_ignores_non_cmd_keys_and_orders_numerically():
    experience = {'config': {}, 'cmd01': {}, 'cmd00': {}, 'cmd10': {}, 'cmd02': {}}
    assert sorted_cmd_keys(experience) == ['cmd00', 'cmd01', 'cmd02', 'cmd10']


def test_time_trigger_waits_for_the_configured_delay():
    trigger = {'type': 'time', 'value': 10.0}
    assert trigger_satisfied(trigger, 9.9, {}) is False
    assert trigger_satisfied(trigger, 10.0, {}) is True
    assert trigger_satisfied(trigger, 15.0, {}) is True


def test_topic_trigger_waits_for_the_exact_value():
    trigger = {'type': 'topic', 'name': 'swarm/status', 'value': 'init'}
    assert trigger_satisfied(trigger, 0.0, {}) is False
    assert trigger_satisfied(trigger, 0.0, {'swarm/status': 'flying'}) is False
    assert trigger_satisfied(trigger, 0.0, {'swarm/status': 'init'}) is True


def test_unknown_trigger_type_is_never_satisfied():
    assert trigger_satisfied({'type': 'bogus'}, 999.0, {}) is False
