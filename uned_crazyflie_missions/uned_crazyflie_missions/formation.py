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
Mission node: one instance per robot, drives a distance/offset consensus formation law.

Subscribes to its own pose and its neighbours' poses,
publishes its own absolute goal pose -- never talks to cflib/Webots
directly, just topics (same contract as waypoints.py/uned_crazyflie_driver).

The control law itself is not new: adapted from the old
shape_based_formation_control.py's task_manager (which computed it inline,
tangled with a direct cflib swarm connection this package no longer needs
-- uned_crazyflie_driver's swarm_driver/webots_driver already do that
part). For each neighbour, the desired relative offset (dx, dy, dz) minus
the actual relative offset is the error; the correction is the mean error
across neighbours plus a running integral term, added to the robot's own
current position to produce an absolute goal:

    correction_axis = mean(desired_offset - actual_offset over neighbours)
    integral_axis += correction_axis * period / integral_divisor_axis
    goal_axis = current_axis + correction_axis + integral_axis

integral_divisor_axis defaults to 1.0 for x/y and 5.0 for z (gentler
altitude correction), matching the original law's tuning -- configurable
per this node's own config, not hardcoded. One real difference from the
original: the original updated its integral term using the *previous*
iteration's error (an artifact of updating `self.x_error` after the
integral step, not obviously intentional) -- this version uses the
current iteration's error, which is simpler and not worse; flagged here
rather than silently replicating what looked like an off-by-one-iteration
quirk.

This config schema (topics, neighbour list, gains) is this node's own
design, since no reference example was given for it (unlike
waypoints.py/sequencer.py, which mirror real example .yaml files already
in uned_crazyflie_config/resources/) -- flagged in the README for review.

    config:
      output: '/dron01/goal_pose'
      input: '/dron01/local_pose'
      period: 0.02

    neighbours:
      N0: {id: dron02, topic: '/dron02/local_pose', dx: 0.25, dy: 0.25, dz: 0.1}
      N1: {id: dron03, topic: '/dron03/local_pose', dx: -0.25, dy: 0.25, dz: 0.1}

    gains:
      integral_divisor_xy: 1.0
      integral_divisor_z: 5.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml


def compute_correction(own_xyz, neighbour_offsets):
    """
    Mean, per axis, of (desired_offset - actual_relative_position) over neighbours.

    own_xyz: (x, y, z) of this robot.
    neighbour_offsets: list of (desired_dx, desired_dy, desired_dz, neighbour_x,
        neighbour_y, neighbour_z).
    Returns (dx, dy, dz), both 0 if there are no neighbours.
    """
    if not neighbour_offsets:
        return 0.0, 0.0, 0.0
    ox, oy, oz = own_xyz
    sx = sy = sz = 0.0
    for dx, dy, dz, nx, ny, nz in neighbour_offsets:
        sx += dx - (ox - nx)
        sy += dy - (oy - ny)
        sz += dz - (oz - nz)
    n = len(neighbour_offsets)
    return sx / n, sy / n, sz / n


class FormationNode(Node):
    def __init__(self):
        super().__init__('formation')

        self.declare_parameter('config', '')
        config_path = self.get_parameter('config').get_parameter_value().string_value
        if not config_path:
            raise RuntimeError("formation: the 'config' parameter (path to a .yaml) is required")
        with open(config_path, 'r') as f:
            experience = yaml.safe_load(f)

        cfg = experience['config']
        self.output_topic = cfg['output']
        self.input_topic = cfg['input']
        self.period = float(cfg.get('period', 0.02))

        gains = experience.get('gains', {})
        self.integral_divisor_xy = float(gains.get('integral_divisor_xy', 1.0))
        self.integral_divisor_z = float(gains.get('integral_divisor_z', 5.0))

        self.own_pose = None
        self.neighbour_poses = {}
        self.neighbour_offsets = {}
        self.integral = [0.0, 0.0, 0.0]

        for entry in experience.get('neighbours', {}).values():
            topic = entry['topic']
            self.neighbour_offsets[topic] = (
                float(entry.get('dx', 0.0)), float(entry.get('dy', 0.0)),
                float(entry.get('dz', 0.0)))
            self.neighbour_poses[topic] = None
            self.create_subscription(
                PoseStamped, topic, self._make_neighbour_callback(topic), 10)

        self.create_subscription(PoseStamped, self.input_topic, self._own_pose_callback, 10)
        self.publisher_goal = self.create_publisher(PoseStamped, self.output_topic, 10)

        self.timer = self.create_timer(self.period, self.iterate)
        self.get_logger().info(
            'formation: output=%s input=%s neighbours=%d' %
            (self.output_topic, self.input_topic, len(self.neighbour_offsets)))

    def _own_pose_callback(self, msg):
        self.own_pose = msg.pose

    def _make_neighbour_callback(self, topic):
        def callback(msg):
            self.neighbour_poses[topic] = msg.pose
        return callback

    def iterate(self):
        if self.own_pose is None:
            return

        neighbour_offsets = []
        for topic, (dx, dy, dz) in self.neighbour_offsets.items():
            pose = self.neighbour_poses[topic]
            if pose is None:
                continue
            neighbour_offsets.append(
                (dx, dy, dz, pose.position.x, pose.position.y, pose.position.z))

        own_xyz = (self.own_pose.position.x, self.own_pose.position.y,
                   self.own_pose.position.z)
        cx, cy, cz = compute_correction(own_xyz, neighbour_offsets)

        self.integral[0] += cx * self.period / self.integral_divisor_xy
        self.integral[1] += cy * self.period / self.integral_divisor_xy
        self.integral[2] += cz * self.period / self.integral_divisor_z

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = own_xyz[0] + cx + self.integral[0]
        msg.pose.position.y = own_xyz[1] + cy + self.integral[1]
        msg.pose.position.z = own_xyz[2] + cz + self.integral[2]
        msg.pose.orientation.w = 1.0
        self.publisher_goal.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FormationNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
