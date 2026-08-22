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
Mission node: drive a robot through a sequence of waypoints read from a
config file, publishing an absolute goal pose and reading a feedback pose
-- both topics/types/fields fully configurable in the yaml, so this node
never hardcodes which robot or driver it's talking to. Works the same with
a physical or virtual Crazyflie: it only talks over topics, the same
contract already used by uned_crazyflie_driver (swarm_driver/webots_driver)
-- an "order" topic (std_msgs/String, 'take_off'/'land') plus the
configured output/input pose topics.

Config schema (see uned_crazyflie_config/resources/demo_individual_waypoints.yaml
for a real example):

    config:
      order: '/dron01/order'      # optional, default: <output's namespace>/order
      output: '/dron01/goal_pose'
      output_type: 'PoseStamped'  # only PoseStamped implemented today
      output_field: 'full'        # only 'full' implemented today (position + orientation)
      input: '/dron01/local_pose'
      input_type: 'PoseStamped'   # only PoseStamped implemented today
      input_field: 'full'         # only 'full' implemented today
      robot: 'dron'                # free-form label, used only for logging
      range: 0.5                  # meters, waypoint-reached tolerance
      shape: 'polygon'            # 'polygon': visit in the given order
                                    # 'tsp': solve the visiting order (tsp.py)
      repeat: 1                   # 0 = loop forever, N = run the sequence N times
      period: 1.0                 # control-loop period, seconds

    points:
      P0: {x: 0.0, y: 1.0, z: 1.0, t: 5.0}   # t: per-point timeout (seconds);
      P1: {x: 0.866, y: 0.5, z: 1.0, t: 5.0}  # move on even if not reached
      ...

'output_field'/'input_field' and the exact 'robot' semantics aren't
specified anywhere else in this repo -- this is this node's own reading of
Francisco's config schema, flagged in the README for review rather than
guessed silently.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import yaml

from uned_crazyflie_missions.tsp import solve_tsp

STATE_INIT = 'init'
STATE_TAKEOFF = 'takeoff'
STATE_EN_ROUTE = 'en_route'
STATE_LANDING = 'landing'
STATE_DONE = 'done'

TAKEOFF_SETTLE_S = 4.0   # same settle time take_off()/_ready() use in the drivers
LANDING_SETTLE_S = 3.0

SUPPORTED_TYPES = {'PoseStamped': PoseStamped}


def load_points(points_config):
    """Turn the yaml 'points' dict into an ordered list of (x, y, z, t) tuples."""
    points = []
    for _, p in points_config.items():
        points.append((float(p['x']), float(p['y']), float(p['z']), float(p.get('t', 0.0))))
    return points


def order_for_shape(shape, points):
    """Return the visiting order (list of indices) for the given 'shape' strategy."""
    xyz = [(p[0], p[1], p[2]) for p in points]
    if shape == 'tsp':
        if len(xyz) < 2:
            return list(range(len(xyz)))
        return solve_tsp(xyz, start=0)
    # 'polygon' (default): visit in the order the points were declared.
    return list(range(len(points)))


class WaypointsNode(Node):
    def __init__(self):
        super().__init__('waypoints')

        self.declare_parameter('config', '')
        config_path = self.get_parameter('config').get_parameter_value().string_value
        if not config_path:
            raise RuntimeError("waypoints: the 'config' parameter (path to a .yaml) is required")
        with open(config_path, 'r') as f:
            experience = yaml.safe_load(f)

        cfg = experience['config']
        self.output_topic = cfg['output']
        self.input_topic = cfg['input']
        self.output_type_name = cfg.get('output_type', 'PoseStamped')
        self.input_type_name = cfg.get('input_type', 'PoseStamped')
        if self.output_type_name not in SUPPORTED_TYPES or \
                self.input_type_name not in SUPPORTED_TYPES:
            raise RuntimeError(
                "waypoints: only 'PoseStamped' is implemented for output_type/input_type, "
                'got %s/%s' % (self.output_type_name, self.input_type_name))
        self.robot_label = cfg.get('robot', '')
        self.tolerance = float(cfg.get('range', 0.15))
        self.shape = cfg.get('shape', 'polygon')
        self.repeat = int(cfg.get('repeat', 1))
        self.period = float(cfg.get('period', 0.2))
        self.order_topic = cfg.get('order', self.output_topic.rsplit('/', 1)[0] + '/order')

        self.points = load_points(experience.get('points', {}))
        self.visit_order = order_for_shape(self.shape, self.points) if self.points else []
        self.get_logger().info(
            'waypoints [%s]: %d points, shape=%s, order=%s' %
            (self.robot_label, len(self.points), self.shape, self.visit_order))

        self.current_pose = None
        self.state = STATE_INIT
        self.target_index = 0
        self.run_count = 0
        self.state_entered_at = self.get_clock().now()

        self.publisher_order = self.create_publisher(String, self.order_topic, 10)
        self.publisher_goal_pose = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.sub_local_pose = self.create_subscription(
            PoseStamped, self.input_topic, self.local_pose_callback, 10)

        self.timer = self.create_timer(self.period, self.iterate)

    def local_pose_callback(self, msg):
        self.current_pose = msg.pose

    def _elapsed_since_state_entered(self):
        return (self.get_clock().now() - self.state_entered_at).nanoseconds / 1e9

    def _enter_state(self, state):
        self.get_logger().info('waypoints [%s]: %s -> %s' % (self.robot_label, self.state, state))
        self.state = state
        self.state_entered_at = self.get_clock().now()

    def _publish_goal(self, index):
        x, y, z, _t = self.points[self.visit_order[index]]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.publisher_goal_pose.publish(msg)
        self.get_logger().info(
            'waypoints [%s]: waypoint %d/%d -> X:%.2f Y:%.2f Z:%.2f' %
            (self.robot_label, index + 1, len(self.visit_order), x, y, z))

    def _reached_or_timed_out(self, index):
        x, y, z, t = self.points[self.visit_order[index]]
        if t > 0.0 and self._elapsed_since_state_entered() >= t:
            return True
        if self.current_pose is None:
            return False
        dx = self.current_pose.position.x - x
        dy = self.current_pose.position.y - y
        dz = self.current_pose.position.z - z
        return (dx * dx + dy * dy + dz * dz) ** 0.5 <= self.tolerance

    def iterate(self):
        if not self.visit_order:
            return

        if self.state == STATE_INIT:
            msg = String()
            msg.data = 'take_off'
            self.publisher_order.publish(msg)
            self._enter_state(STATE_TAKEOFF)

        elif self.state == STATE_TAKEOFF:
            if self._elapsed_since_state_entered() >= TAKEOFF_SETTLE_S:
                self.target_index = 0
                self._publish_goal(self.target_index)
                self._enter_state(STATE_EN_ROUTE)

        elif self.state == STATE_EN_ROUTE:
            if self._reached_or_timed_out(self.target_index):
                if self.target_index + 1 < len(self.visit_order):
                    self.target_index += 1
                    self._publish_goal(self.target_index)
                    self.state_entered_at = self.get_clock().now()
                else:
                    self.run_count += 1
                    if self.repeat == 0 or self.run_count < self.repeat:
                        self.target_index = 0
                        self._publish_goal(self.target_index)
                        self.state_entered_at = self.get_clock().now()
                    else:
                        msg = String()
                        msg.data = 'land'
                        self.publisher_order.publish(msg)
                        self._enter_state(STATE_LANDING)

        elif self.state == STATE_LANDING:
            if self._elapsed_since_state_entered() >= LANDING_SETTLE_S:
                self._enter_state(STATE_DONE)
                self.get_logger().info('waypoints [%s]: mission complete.' % self.robot_label)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
