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
Nodo de misión: recorrido de waypoints resolviendo el orden como un TSP.

Usa un heurístico simple (ver tsp.py). Funciona igual con un Crazyflie
físico o virtual (Webots): solo usa el contrato de topics ya compartido
por uned_crazyflie_driver (crazyflie_agent.py) y uned_crazyflie_webots
(crazyflie_driver.py) -- <robot_id>/order (std_msgs/String,
'take_off'/'land') y <robot_id>/goal_pose (geometry_msgs/PoseStamped)
como comando, leyendo <robot_id>/local_pose (geometry_msgs/PoseStamped)
como realimentación -- no habla directamente con cflib ni con Webots.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from uned_crazyflie_missions.tsp import solve_tsp

STATE_INIT = 'init'
STATE_TAKEOFF = 'takeoff'
STATE_EN_ROUTE = 'en_route'
STATE_LANDING = 'landing'
STATE_DONE = 'done'

TAKEOFF_SETTLE_S = 4.0   # mismo tiempo que usan take_off()/_ready() en los drivers
LANDING_SETTLE_S = 3.0


class TSPWaypointsNode(Node):
    def __init__(self):
        super().__init__('tsp_waypoints')

        self.declare_parameter('robot_id', 'dron01')
        self.declare_parameter('waypoints', [0.0, 0.0, 1.0])
        self.declare_parameter('tolerance', 0.15)
        self.declare_parameter('loop', False)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        flat = self.get_parameter('waypoints').get_parameter_value().double_array_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value

        if len(flat) < 3 or len(flat) % 3 != 0:
            self.get_logger().error(
                'tsp_waypoints: el parámetro "waypoints" debe tener 3*N valores '
                '(x1,y1,z1, x2,y2,z2, ...), tiene %d' % len(flat))
            self.points = []
        else:
            self.points = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        self.visit_order = []
        if len(self.points) >= 2:
            self.visit_order = solve_tsp(self.points, start=0)
            self.get_logger().info(
                'tsp_waypoints: %d waypoints, orden de visita: %s' %
                (len(self.points), self.visit_order))
        elif len(self.points) == 1:
            self.visit_order = [0]

        self.current_pose = None
        self.state = STATE_INIT
        self.target_index = 0
        self.state_entered_at = self.get_clock().now()

        self.publisher_order = self.create_publisher(String, self.robot_id + '/order', 10)
        self.publisher_goal_pose = self.create_publisher(
            PoseStamped, self.robot_id + '/goal_pose', 10)
        self.sub_local_pose = self.create_subscription(
            PoseStamped, self.robot_id + '/local_pose', self.local_pose_callback, 10)

        self.timer = self.create_timer(0.2, self.iterate)

    def local_pose_callback(self, msg):
        self.current_pose = msg.pose

    def _elapsed_since_state_entered(self):
        return (self.get_clock().now() - self.state_entered_at).nanoseconds / 1e9

    def _enter_state(self, state):
        self.get_logger().info('tsp_waypoints: %s -> %s' % (self.state, state))
        self.state = state
        self.state_entered_at = self.get_clock().now()

    def _publish_goal(self, index):
        x, y, z = self.points[self.visit_order[index]]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.publisher_goal_pose.publish(msg)
        self.get_logger().info(
            'tsp_waypoints: waypoint %d/%d -> X:%.2f Y:%.2f Z:%.2f' %
            (index + 1, len(self.visit_order), x, y, z))

    def _reached(self, x, y, z):
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
            x, y, z = self.points[self.visit_order[self.target_index]]
            if self._reached(x, y, z):
                if self.target_index + 1 < len(self.visit_order):
                    self.target_index += 1
                    self._publish_goal(self.target_index)
                elif self.loop:
                    self.target_index = 0
                    self._publish_goal(self.target_index)
                else:
                    msg = String()
                    msg.data = 'land'
                    self.publisher_order.publish(msg)
                    self._enter_state(STATE_LANDING)

        elif self.state == STATE_LANDING:
            if self._elapsed_since_state_entered() >= LANDING_SETTLE_S:
                self._enter_state(STATE_DONE)
                self.get_logger().info('tsp_waypoints: misión completada.')


def main(args=None):
    rclpy.init(args=args)
    node = TSPWaypointsNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
