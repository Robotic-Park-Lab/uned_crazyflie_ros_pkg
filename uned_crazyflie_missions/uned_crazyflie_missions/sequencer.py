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
Mission node: publish a scripted sequence of values to arbitrary topics,
each step gated either by a fixed delay or by waiting for a specific value
to appear on a subscribed topic. Generic demo/experiment choreography --
doesn't know about Crazyflies specifically, just topics.

Config schema (see uned_crazyflie_config/resources/demo_individual_waypoints_topics.yaml
for a real example):

    config:
      publisher:
        topic00: {name: 'swarm/order', type: 'String'}
        ...
      subscription:
        topic00: {name: 'swarm/status', type: 'String'}
        ...

    cmd00:
      topic: 'swarm/order'          # must match a config.publisher name
      type: 'String'
      value: 'take_off'
      trigger:
        type: 'topic'               # wait until config.subscription <name> == value
        name: 'swarm/status'
        value: 'init'
    cmd01:
      topic: 'swarm/order'
      type: 'String'
      value: 'formation_run'
      trigger:
        type: 'time'                # wait <value> seconds after the previous
        value: 10.0                 # command ran (or after node start, for cmd00)

Commands run in ascending key order (cmd00, cmd01, cmd02, ...). Only
std_msgs/String is implemented today, since it's the only type used in
Francisco's reference config -- flagged here and in the README, not
silently limited.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

SUPPORTED_TYPES = {'String': String}


def sorted_cmd_keys(experience):
    """Command dict keys ('cmd00', 'cmd01', ...) in run order."""
    return sorted(k for k in experience.keys() if k.startswith('cmd'))


def trigger_satisfied(trigger, elapsed_s, last_value_by_topic):
    """
    Pure trigger check, no ROS/clock dependency.

    trigger: {'type': 'time', 'value': <seconds>} or
        {'type': 'topic', 'name': <topic>, 'value': <expected>}.
    elapsed_s: seconds since the previous command ran (ignored for 'topic' triggers).
    last_value_by_topic: {topic: last received value}.
    """
    if trigger['type'] == 'time':
        return elapsed_s >= float(trigger['value'])
    if trigger['type'] == 'topic':
        return last_value_by_topic.get(trigger['name']) == trigger['value']
    return False


class SequencerNode(Node):
    def __init__(self):
        super().__init__('sequencer')

        self.declare_parameter('config', '')
        config_path = self.get_parameter('config').get_parameter_value().string_value
        if not config_path:
            raise RuntimeError("sequencer: the 'config' parameter (path to a .yaml) is required")
        with open(config_path, 'r') as f:
            experience = yaml.safe_load(f)

        cfg = experience.get('config', {})
        self.publishers_by_topic = {}
        for entry in cfg.get('publisher', {}).values():
            msg_type = SUPPORTED_TYPES.get(entry['type'])
            if msg_type is None:
                raise RuntimeError(
                    "sequencer: unsupported publisher type '%s' (only String is implemented)"
                    % entry['type'])
            self.publishers_by_topic[entry['name']] = self.create_publisher(
                msg_type, entry['name'], 10)

        self.last_value_by_topic = {}
        for entry in cfg.get('subscription', {}).values():
            msg_type = SUPPORTED_TYPES.get(entry['type'])
            if msg_type is None:
                raise RuntimeError(
                    "sequencer: unsupported subscription type '%s' (only String is implemented)"
                    % entry['type'])
            topic = entry['name']
            self.last_value_by_topic[topic] = None
            self.create_subscription(
                msg_type, topic, self._make_sub_callback(topic), 10)

        self.commands = [experience[k] for k in sorted_cmd_keys(experience)]
        self.get_logger().info('sequencer: %d commands loaded' % len(self.commands))

        self.next_index = 0
        self.waiting_since = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.iterate)

    def _make_sub_callback(self, topic):
        def callback(msg):
            self.last_value_by_topic[topic] = msg.data
        return callback

    def _elapsed_since_waiting(self):
        return (self.get_clock().now() - self.waiting_since).nanoseconds / 1e9

    def iterate(self):
        if self.next_index >= len(self.commands):
            return

        cmd = self.commands[self.next_index]
        if cmd['trigger']['type'] not in ('time', 'topic'):
            self.get_logger().error(
                "sequencer: unknown trigger type '%s'" % cmd['trigger']['type'])
            return
        if not trigger_satisfied(
                cmd['trigger'], self._elapsed_since_waiting(), self.last_value_by_topic):
            return

        publisher = self.publishers_by_topic.get(cmd['topic'])
        if publisher is None:
            self.get_logger().error(
                "sequencer: command targets undeclared topic '%s'" % cmd['topic'])
        else:
            msg = String()
            msg.data = cmd['value']
            publisher.publish(msg)
            self.get_logger().info(
                'sequencer: published %r on %s' % (cmd['value'], cmd['topic']))

        self.next_index += 1
        self.waiting_since = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = SequencerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
