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

from std_msgs.msg import String, Bool, Float64, Float64MultiArray, MultiArrayDimension, UInt16MultiArray
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from uned_crazyflie_driver.pid_controller import PIDController
from uned_crazyflie_driver.agent import Agent
from multi_agent_pkg.lagrange_multipliers import Sphere, Cone, Ellipsoid

from math import atan2, cos, sin, degrees, pi, sqrt


def load_formation_params(parent):
    parent.k = parent.driver_cfg['task_contr_gain']
    aux = parent.driver_cfg['task_contr_ul']
    parent.ul = sqrt(3 * pow(aux, 2))
    aux = parent.driver_cfg['task_contr_ll']
    parent.ll = -sqrt(3 * pow(aux, 2))

    parent.node.destroy_subscription(parent.subscriber['goal_pose'])
    parent.publisher['goal_pose'] = parent.node.create_publisher(PoseStamped, parent.id + '/goal_pose', 10)
    parent.publisher['global_error'] = parent.node.create_publisher(Float64, parent.id + '/global_error', 10)
    parent.node.get_logger().debug('Task %s by %s' % (parent.driver_cfg['task_type'], parent.driver_cfg['task_role']))

    parent.publisher['event_fx'] = parent.node.create_publisher(Bool, parent.id + '/formation/event_x', 10)
    parent.publisher['event_fy'] = parent.node.create_publisher(Bool, parent.id + '/formation/event_y', 10)
    parent.publisher['event_fz'] = parent.node.create_publisher(Bool, parent.id + '/formation/event_z', 10)

    if parent.driver_cfg['task_contr_type'] == 'pid':
        parent.formation_x_controller = PIDController(parent.k, 0.0, 0.0, 0.0, 100, parent.ul, parent.ll, parent.trigger_ai, parent.trigger_co)
        parent.formation_y_controller = PIDController(parent.k, 0.0, 0.0, 0.0, 100, parent.ul, parent.ll, parent.trigger_ai, parent.trigger_co)
        parent.formation_z_controller = PIDController(parent.k, 0.0, 0.0, 0.0, 100, parent.ul, parent.ll, parent.trigger_ai, parent.trigger_co)

    parent.agent_list = list()
    aux = parent.config['task']['relationship']
    parent.relationship = aux.split(', ')
    if parent.driver_cfg['task_type'] == 'distance':
        if any(t in parent.driver_cfg['task_contr_type'] for t in ('gradient', 'ML1', 'ML2', 'ML3')):
            parent.node.create_timer(parent.driver_cfg['task_contr_period'], parent.distance_gradient_controller)
        for rel in parent.relationship:
            parent.N = parent.N + 1.0
            aux = rel.split('_')
            id = aux[0]

            if not id.find("line") == -1:
                p = Point()
                p.x = float(aux[1])
                p.y = float(aux[2])
                p.z = float(aux[3])
                u = Vector3()
                u.x = float(aux[4])
                u.y = float(aux[5])
                u.z = float(aux[6])
                robot = Agent(parent, parent.node, id, point=p, vector=u)
                parent.node.get_logger().debug(
                    'Agent: %s: Neighbour: %s ::: Px: %s Py: %s Pz: %s' %
                    (parent.id, id, aux[1], aux[2], aux[3]))
            else:
                if aux[0] == 'origin' or aux[0] == 'sphere':
                    auxa = aux[1]
                    auxb = auxa.split('-')
                    origin = Point()
                    origin.x = float(auxb[1])
                    origin.y = float(auxb[2])
                    origin.z = float(auxb[3])
                    parent.R = float(auxb[0])
                    parent.geometry = Sphere(parent.R, origin)
                    robot = Agent(parent, parent.node, aux[0], d=float(auxb[0]), point=origin)
                elif aux[0] == 'cone':
                    auxa = aux[1]
                    auxb = auxa.split('-')
                    origin = Point()
                    origin.x = float(auxb[2])
                    origin.y = float(auxb[3])
                    origin.z = float(auxb[4])
                    parent.geometry = Cone(float(auxb[0]), float(auxb[1]), origin)
                    parent.node.get_logger().info(
                        'Agent: %s: Cono: a %s \tc: %s' %
                        (parent.id, auxb[0], auxb[1]))
                    robot = Agent(parent, parent.node, aux[0], d=float(auxb[0]), point=origin)
                elif aux[0] == 'ellipsoid':
                    auxa = aux[1]
                    auxb = auxa.split('-')
                    origin = Point()
                    origin.x = float(auxb[0])
                    origin.y = float(auxb[1])
                    origin.z = float(auxb[2])
                    auxa = aux[2]
                    auxb = auxa.split('-')
                    a = float(auxb[0])
                    b = float(auxb[1])
                    c = float(auxb[2])
                    parent.geometry = Ellipsoid(a=a, b=b, c=c, origin=origin)
                    robot = Agent(parent, parent.node, aux[0], a=a, b=b, c=c, point=origin, d=0.0)
                else:
                    robot = Agent(parent, parent.node, aux[0], d=float(aux[1]))
                parent.node.get_logger().debug(
                    'Agent: %s: Neighbour: %s \td: %s' %
                    (parent.id, aux[0], aux[1]))
            parent.agent_list.append(robot)
    elif parent.driver_cfg['task_type'] == 'pose':
        if any(t in parent.driver_cfg['task_contr_type'] for t in ('gradient', 'ML1', 'ML2', 'ML3')):
            parent.node.create_timer(parent.controller['period'], parent.pose_gradient_controller)
        for rel in parent.relationship:
            aux = rel.split('_')
            robot = Agent(
                parent, parent.node, aux[0], x=float(
                    aux[1]), y=float(
                    aux[2]), z=float(
                    aux[3]))
            parent.node.get_logger().debug(
                'Agent: %s. Neighbour %s :::x: %s \ty: %s \tz: %s' %
                (parent.id, aux[0], aux[1], aux[2], aux[3]))
            parent.agent_list.append(robot)