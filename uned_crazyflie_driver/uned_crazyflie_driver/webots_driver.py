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


import rclpy
from rclpy.time import Time
from threading import Timer
import copy
import yaml

from std_msgs.msg import String, Bool, Float64, Float64MultiArray, MultiArrayDimension, UInt16MultiArray
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path

from math import atan2, cos, sin, degrees, pi, sqrt
import numpy as np
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from uned_crazyflie_driver.agent import Agent
from multi_agent_pkg.lagrange_multipliers import Sphere, Cone, Ellipsoid
from uned_crazyflie_driver.webots_bootstrap import init_webots_devices, init_webots_cascade_controllers
from uned_crazyflie_driver.driver_config import resolve_driver_config, resolve_ros2_config, resolve_variable_config
from uned_crazyflie_driver.sensors import build_laserscan, agent_removal_marker
from uned_crazyflie_driver.multi_agent_config import load_formation_params

# Change this path to your crazyflie-firmware folder
# sys.path.append('/home/kiko/Code/crazyflie-firmware')
# import cffirmware

class PIDController():
    def __init__(self, Kp, Ki, Kd, Td, Nd, UpperLimit, LowerLimit, ai, co):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Td = Td
        self.Nd = Nd
        self.UpperLimit = UpperLimit
        self.LowerLimit = LowerLimit
        self.integral = 0
        self.derivative = 0
        self.error = [0.0, 0.0]
        self.trigger_ai = ai
        self.trigger_co = co
        self.trigger_last_signal = 0.0
        self.noise = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.past_time = 0.0
        self.last_value = 0.0
        self.th = 0.0
        self.rele = False
        self.range = 20.0
        self.cmd = 60.0

    def update(self, dt):
        P = self.Kp * self.error[0]
        self.integral = self.integral + self.Ki*self.error[1]*dt
        self.derivative = (self.Td/(self.Td+self.Nd+dt))*self.derivative+(self.Kd*self.Nd/(self.Td+self.Nd*dt))*(self.error[0]-self.error[1])
        out = P + self.integral + self.derivative
        
        if not self.UpperLimit==0.0:
            # out_i = out
            if out>self.UpperLimit:
                out = self.UpperLimit
            if out<self.LowerLimit:
                out = self.LowerLimit

            # self.integral = self.integral - (out-out_i) * sqrt(self.Kp/self.Ki)
        
        self.error[1] = self.error[0]

        self.last_value = out
        
        return out

    def eval_threshold(self, signal, ref):
        # Noise (Cn)
        mean = signal/len(self.noise)
        for i in range(0,len(self.noise)-2):
            self.noise[i] = self.noise[i+1]
            mean += self.noise[i]/len(self.noise)
        
        self.noise[len(self.noise)-1] = signal

        trigger_cn = 0.0
        for i in range(0,len(self.noise)-1):
            if abs(self.noise[i]-mean) > trigger_cn:
                trigger_cn = self.noise[i]-mean
        trigger_cn = 0.0
        # a
        a = self.trigger_ai * abs(signal - ref)
        if a > self.trigger_ai:
            a = self.trigger_ai

        # Threshold
        self.th = self.trigger_co + a + trigger_cn
        self.inc = abs(abs(ref-signal) - self.trigger_last_signal) 
        # Delta Error
        if (self.inc >= abs(self.th)):
            self.trigger_last_signal = abs(ref-signal)
            return True

        return False


#####################################
# Virtual Crazyflie Logging Class ##
#####################################
class CrazyflieWebotsDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())

        # Read config_file
        self.config_file = properties.get("config_file")
        with open(self.config_file, 'r') as file:
            documents = yaml.safe_load(file)
        for robot in documents['Robots']:
            if documents['Robots'][robot]['name'] == properties.get("name_id"):
                self.config = documents['Robots'][robot]
        # Init ROS2 Node
        self.id = self.config['name']
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.id + '_driver')

        # self.crazyflie = Crazyflie_ROS2(self, self.node, self.id, self.id, self.config, webots_node=webots_node)

        # Intialize Variables
        self.variables = resolve_variable_config()
        self.past_time = self.robot.getTime()
    
        # Lectura de parámetros de configuración
        self.driver_cfg = resolve_driver_config(self.config, self.node.get_logger())
        
        # Intialize Crazyflie configuration
        # Motores, sensores y los 12 PID en cascada: compartidos con
        self.__dict__.update(init_webots_devices(self.robot, timestep))
        self.__dict__.update(init_webots_cascade_controllers())
        
        self.led_ring.set(1)

        self.initialize()

    def initialize(self):
        self.node.get_logger().info('Connected to Webots -> Crazyflie %s' % self.id)

        # Configuración de los topics comunes a crazyflies reales y virtuales
        self.publisher, self.subscriber = resolve_ros2_config(self)
        self.tfbr = TransformBroadcaster(self.node)
        self.node.create_timer(0.2, self.publish_laserscan_data)

        if self.driver_cfg['task_enable'] or self.driver_cfg['task_onboard']:
            load_formation_params(self)

        self.node.get_logger().info('Webots_Node::inicialize() ok. %s' % (str(self.id)))
        msg = String()
        msg.data = 'init'
        self.publisher['swarm_status'].publish(msg)

    def publish_laserscan_data(self):
        # Compartido con Crazyflie_ROS2.publish_laserscan() vía
        # uned_crazyflie_driver.sensors.build_laserscan (ver AUDIT.md rama doc).
        front_range = self.range_front.getValue() / 1000.0
        back_range = self.range_back.getValue() / 1000.0
        left_range = self.range_left.getValue() / 1000.0
        right_range = self.range_right.getValue() / 1000.0
        self.msg_laser = build_laserscan(front_range, back_range, left_range, right_range, Time(seconds=self.robot.getTime()).to_msg(), self.id)
        self.publisher['laser'].publish(self.msg_laser)

    def dt_pose_callback(self, pose):
        self.node.get_logger().debug('TO-DO: DT Pose: X:%f Y:%f' %(pose.pose.position.x, pose.pose.position.y))
        # self.robot.getSelf().getField("translation").setSFVec3f([pose.position.x,
        # pose.position.y, pose.position.z])
        # self.robot.getSelf().getField("rotation").setSFVec3f([0.0, 0.0, 0.0])

    def cmd_vel_callback(self, twist):
        self.variables['target_twist'] = twist

    def goal_pose_callback(self, pose):
        self.variables['target_pose'] = pose
        if self.variables['target_pose'].pose.position.z < 0.6:
            self.variables['target_pose'].pose.position.z = 0.6
        elif self.variables['target_pose'].pose.position.z > 3.0:
            self.variables['target_pose'].pose.position.z = 3.0

    def order_callback(self, msg):
        self.node.get_logger().info('Order: "%s"' % msg.data)
        if msg.data == 'take_off':
            if self.variables['_is_flying']:
                self.node.get_logger().warning('Already flying')
            else:
                self.take_off()
        elif msg.data == 'land':
            if self.variables['_is_flying']:
                self.descent()
                self.variables['formation'] = False
            else:
                self.node.get_logger().warning('In land')
        elif msg.data == 'formation_run':
            if self.driver_cfg['task_enable']:
                self.variables['formation'] = True
        elif msg.data == 'formation_stop':
            self.variables['formation'] = False
        elif msg.data == 'disconnect':
            self._disconnected()
        elif not msg.data.find("ellipsoid_") == -1:
            aux = msg.data.split('_')
            auxa = aux[1]
            auxb = auxa.split('-')
            self.geometry.a = float(auxb[0])
            self.geometry.b = float(auxb[1])
            self.geometry.c = float(auxb[2])
        elif msg.data == 'reconfiguration':
            self.variables['update_gain'] = True
            self.variables['state'] = [10.0, 10.0, 10.0, 10.0, 10.0]
            for agent in self.agent_list:
                if agent.id == 'origin':
                    agent.k = 4.0
        elif not msg.data.find("remove") == -1 and self.driver_cfg['task_enable']:
            self.remove_agent(msg.data)
        elif not msg.data.find("add") == -1 and self.driver_cfg['task_enable']:
            self.add_agent(msg.data)
        else:
            self.node.get_logger().error('"%s": Unknown order' % (msg.data))

    def swarm_goalpose_callback(self, msg):
        if not self.variables['centroid_leader']:
            self.node.get_logger().info('Formation Control::Leader-> Centroid.')
        self.variables['centroid_leader'] = True
        self.variables['leader_cmd'] = msg

    def take_off(self):
        self.variables['target_pose'].pose.position.z = 1.1
        self.node.get_logger().info('Take Off...x:%.2f; y:%.2f; z:%.2f' % (self.variables['target_pose'].pose.position.x, self.variables['target_pose'].pose.position.y, self.variables['target_pose'].pose.position.z))
        self.variables['_is_flying'] = True
        self.t_ready = Timer(4, self._ready)
        self.t_ready.start()

    def _ready(self):
        self.node.get_logger().info('Ready!!.')
        self.variables['ready'] = True
        if self.id == 'dron01':
            msg = String()
            msg.data = 'ready'
            self.publisher['swarm_status'].publish(msg)

    def descent(self):
        self.variables['target_pose'].pose.position.z = 0.6
        self.node.get_logger().info('Descent...')
        self.t_desc = Timer(2, self.take_land)
        self.variables['_is_flying'] = False
        self.variables['ready'] = False
        self.t_desc.start()

    def take_land(self):
        self.node.get_logger().info('Take Land.')
        self.variables['target_pose'].pose.position.z = 0.1
        self.t_desc = Timer(2, self.stop)
        self.variables['init_pose'] = False

    def stop(self):
        self.variables['target_pose'].pose.position.z = 0.0

    def add_agent(self, data):
        aux = data.split('_')
        robot = Agent(self, self.node, aux[1], d=float(aux[2]))
        self.agent_list.append(robot)

    def remove_agent(self, data):
        aux = data.split('_')
        j = 0
        for agent in self.agent_list:
            if agent.id == aux[1]:
                agent.disconnect = True
                line = agent_removal_marker(self.node.get_clock().now().to_msg())
                agent.publisher_marker_.publish(line)
                agent.parent.node.destroy_publisher(agent.publisher_marker_)
                self.agent_list.pop(j)
            else:
                j += 1

    def _disconnected(self):
        self.node.get_logger().info('TO-DO Disconnect.')
        # self.descent()
        self.variables['target_pose'].pose.position.x = self.variables['target_pose'].pose.position.x * 1.2
        self.variables['target_pose'].pose.position.y = self.variables['target_pose'].pose.position.y * 1.2
        self.variables['target_pose'].pose.position.z = self.variables['target_pose'].pose.position.z * 0.8
        self.variables['disconnect'] = True
        self.node.destroy_publisher(self.publisher['pose_publisher'])
        for agent in self.agent_list:
            agent.variables['disconnect'] = True
            line = agent_removal_marker(self.node.get_clock().now().to_msg())
            agent.publisher_marker_.publish(line)
            agent.parent.node.destroy_publisher(agent.publisher_marker_)
            msg = String()
            msg.data = 'remove_' + self.id
            agent.publisher_order_.publish(msg)

    ###############
    #    Tasks    #
    ###############
    def distance_gradient_controller(self):
        if self.formation and not self.variables['disconnect']:
            msg_error = Float64()
            msg_error.data = 0.0
            dx = dy = dz = 0
            if 'ML2' in self.controller_type:
                projection = self.geometry.projection(self.pose.position)
                m_gain = self.geometry.R / sqrt(
                    pow(self.pose.position.x - self.geometry.origin.x, 2) +
                    pow(self.pose.position.y - self.geometry.origin.y, 2) +
                    pow(self.pose.position.z - self.geometry.origin.z, 2))
                pxx = m_gain * (1 - pow(projection.x, 2) / pow(self.geometry.R, 2))
                pxy = m_gain * (-projection.x * projection.y / pow(self.geometry.R, 2))
                pxz = m_gain * (-projection.x * (self.pose.position.z -
                                self.geometry.origin.z) / pow(self.geometry.R, 2))
                pyy = m_gain * (1 - pow(projection.y, 2) / pow(self.geometry.R, 2))
                pyz = m_gain * (-projection.y * (self.pose.position.z -
                                self.geometry.origin.z) / pow(self.geometry.R, 2))
                pzz = m_gain * (1 - pow(self.pose.position.z -
                                self.geometry.origin.z, 2) / pow(self.geometry.R, 2))
                '''
                projection = self.geometry.projection(self.pose.position)
                fpi = self.geometry.value1(projection)
                # P = I - p_i*p_i^T/self.geometry.value
                pxx = 1-(pow(projection.x,2)/pow(self.geometry.a,4))/fpi
                pxy = (-projection.x*projection.y/pow(self.geometry.a,4))/fpi
                pxz = (projection.x*(projection.z-self.geometry.c)/(pow(self.geometry.a,2)*
                    pow(self.geometry.c,2)))/fpi
                pyy = 1-(pow(projection.y,2)/pow(self.geometry.a,4))/fpi
                pyz = (projection.y*(projection.z-self.geometry.c)/(pow(self.geometry.a,2)*
                    pow(self.geometry.c,2)))/fpi
                pzz = 1-(pow(projection.z-self.geometry.c,2)/pow(self.geometry.c,4))/fpi
                '''
                '''
                if agent.id == 'sphere':
                    # P = I - (pi*pi^T)/R²
                    # px      1 - pix*pix   -pix*piy     -pix*piz
                     # py = (    -piy*pix   1 - piy*piy   -piy*piz  )/R²
                    # pz        -piz*pix    -piz*piy    1 -piz*piz
                    m_gain = self.geometry.R/sqrt(pow(self.pose.position.x-
                        self.geometry.origin.x,2)+pow(self.pose.position.y-self.geometry.origin.y,2)+pow(self.pose.position.z-self.geometry.origin.z,2))
                    pxx = m_gain*(1-pow(projection.x,2)/pow(self.geometry.R,2))
                    pxy = m_gain*(-projection.x*projection.y/pow(self.geometry.R,2))
                    pxz = m_gain*(-projection.x*(self.pose.position.z-
                        self.geometry.origin.z)/pow(self.geometry.R,2))
                    pyy = m_gain*(1-pow(projection.y,2)/pow(self.geometry.R,2))
                    pyz = m_gain*(-projection.y*(self.pose.position.z-
                        self.geometry.origin.z)/pow(self.geometry.R,2))
                    pzz = m_gain*(1-pow(self.pose.position.z-self.geometry.origin.z,2)/
                        pow(self.geometry.R,2))
                if agent.id == 'cone':
                    fpi = self.geometry.value1(projection)
                    # P = I - p_i*p_i^T/self.geometry.value
                    pxx = 1-(pow(projection.x,2)/pow(self.geometry.a,4))/fpi
                    pxy = (-projection.x*projection.y/pow(self.geometry.a,4))/fpi
                    pxz = (projection.x*(projection.z-self.geometry.c)/
                        (pow(self.geometry.a,2)*pow(self.geometry,c,2)))/fpi
                    pyy = 1-(pow(projection.y,2)/pow(self.geometry.a,4))/fpi
                    pyz = (projection.y*(projection.z-self.geometry.c)/
                        (pow(self.geometry.a,2)*pow(self.geometry,c,2)))/fpi
                    pzz = 1-(pow(projection.z-self.c,2)/pow(self.geometry.c,4))/fpi
                '''
            for agent in self.agent_list:
                if not agent.id.find("line") == -1:
                    nearest = PoseStamped()
                    nearest.header.frame_id = "map"
                    gamma = -np.dot([agent.point.x - self.pose.position.x,
                                     agent.point.y - self.pose.position.y,
                                     agent.point.z - self.pose.position.z],
                                    [agent.vector.x,
                                     agent.vector.y,
                                     agent.vector.z]) / agent.mod
                    nearest.pose.position.x = agent.point.x + gamma * agent.vector.x
                    nearest.pose.position.y = agent.point.y + gamma * agent.vector.y
                    nearest.pose.position.z = agent.point.z + gamma * agent.vector.z
                    agent.gtpose_callback(nearest)

                error_x = self.pose.position.x - agent.pose.position.x
                error_y = self.pose.position.y - agent.pose.position.y
                error_z = self.pose.position.z - agent.pose.position.z
                if agent.id == 'origin' or agent.id == 'sphere':
                    distance = pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2)
                    # Gradiente & ML para proyectar el resultado de la ley de control
                    if 'gradient' in self.controller_type or 'ML3' in self.controller_type:
                        dx += - self.k * agent.k * (distance - pow(agent.d, 2)) * error_x
                        dy += - self.k * agent.k * (distance - pow(agent.d, 2)) * error_y
                        dz += - self.k * agent.k * (distance - pow(agent.d, 2)) * error_z
                    # ML como término adicional & ML en consenso + Proyección
                    if 'ML1' in self.controller_type or 'ML2' in self.controller_type:
                        projection = self.geometry.projection(self.pose.position)
                        dx += - self.k * agent.k * (self.pose.position.x - projection.x)
                        dy += - self.k * agent.k * (self.pose.position.y - projection.y)
                        dz += - self.k * agent.k * (self.pose.position.z - projection.z)
                else:
                    if agent.id == 'cone':
                        if 'gradient' in self.controller_type:
                            fpi = self.geometry.value(self.pose.position)
                            k = 0.25
                            dx += - k * self.k * agent.k * fpi * \
                                (error_x / pow(self.geometry.a, 2))
                            dy += - k * self.k * agent.k * fpi * \
                                (error_y / pow(self.geometry.a, 2))
                            dz += k * self.k * agent.k * fpi * \
                                ((error_z - self.geometry.c) / pow(self.geometry.c, 2))
                        # ML como término adicional & ML en consenso + Proyección
                        if 'ML1' in self.controller_type or 'ML2' in self.controller_type:
                            projection = self.geometry.projection(self.pose.position)
                            dx += - self.k * agent.k * (self.pose.position.x - projection.x)
                            dy += - self.k * agent.k * (self.pose.position.y - projection.y)
                            dz += - self.k * agent.k * (self.pose.position.z - projection.z)
                    else:
                        if agent.id == 'ellipsoid':
                            fpi = self.geometry.value(self.pose.position)
                            dx += - self.k * agent.k * fpi * (error_x / pow(self.geometry.a, 2))
                            dy += - self.k * agent.k * fpi * (error_y / pow(self.geometry.b, 2))
                            dz += - self.k * agent.k * fpi * (error_z / pow(self.geometry.c, 2))
                        else:
                            # ML en consenso + Proyección
                            if 'ML2' in self.controller_type:
                                error_x = projection.x - agent.pose.position.x
                                error_y = projection.y - agent.pose.position.y
                                error_z = projection.z - agent.pose.position.z

                                distance = pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2)
                                x = - self.k * agent.k * (distance - pow(agent.d, 2)) * error_x
                                y = - self.k * agent.k * (distance - pow(agent.d, 2)) * error_y
                                z = - self.k * agent.k * (distance - pow(agent.d, 2)) * error_z

                                dx1 = pxx * x + pxy * y + pxz * z
                                dy1 = pxy * x + pyy * y + pyz * z
                                dz1 = pxz * x + pyz * y + pzz * z
                                dx += dx1
                                dy += dy1
                                dz += dz1
                            else:
                                # COMUN
                                distance = pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2)
                                dx += - self.k * agent.k * (distance - pow(agent.d, 2)) * error_x
                                dy += - self.k * agent.k * (distance - pow(agent.d, 2)) * error_y
                                dz += - self.k * agent.k * (distance - pow(agent.d, 2)) * error_z

                if not self.driver_cfg['digital_twin']:
                    msg_data = Float64()
                    aux = sqrt(distance)
                    msg_data.data = aux
                    agent.publisher_data_.publish(msg_data)
                    # error = abs(aux - agent.d)
                    # msg_data.data = agent.last_iae + (agent.last_error + error) *
                    # self.task_period /2
                    # agent.last_error = error
                    # agent.publisher_iae_.publish(msg_data)
                    # agent.last_iae = msg_data.data
                    msg_data.data = distance - pow(agent.d, 2)
                    agent.publisher_error_.publish(msg_data)
                    msg_error.data += msg_data.data
                    # if not self.Fixed_z:
                    # self.node.get_logger().debug('Agent %s: d*: %.2f d: %.2f e^2: %.2f dx: %.2f
                    # dy: %.2f dz: %.2f ' % (agent.id, agent.d, aux, msg_data.data, delta_x,
                    # delta_y, delta_z))
                    # else:
                    # self.node.get_logger().debug('Agent %s: e^2: %.2f dx: %.2f dy: %.2f dz: %.2f
                    # ' % (agent.id, msg_data.data, dx, dy, dz))

            if not self.continuous:
                delta = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
                if not self.eval_threshold(0.0, delta):
                    return

            msg = Bool()
            msg.data = True
            self.event_x.publish(msg)

            # ML en la salida
            if 'ML3' in self.controller_type:
                px = self.pose.position.x + dx
                py = self.pose.position.y + dy
                pz = self.pose.position.z + dz - 0.6
                mod_p = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2))
                px = (px / mod_p) * self.R
                py = (py / mod_p) * self.R
                pz = (pz / mod_p) * self.R + 0.6
                d1 = sqrt(pow(self.pose.position.x - px, 2) +
                          pow(self.pose.position.y - py, 2) + pow(self.pose.position.z - pz, 2))
                d2 = sqrt(pow(self.pose.position.x + px, 2) +
                          pow(self.pose.position.y + py, 2) + pow(self.pose.position.z + pz, 2))
                if d1 < d2:
                    c = 1.0
                else:
                    c = -1.0

                dx = c * px - self.pose.position.x
                dy = c * py - self.pose.position.y
                dz = c * pz - self.pose.position.z

                if '_v' in self.controller_type:
                    dx = (dx - self.pose.position.x)
                    dy = (dy - self.pose.position.y)
                    dz = (dz - self.pose.position.z)

            msg = Float64MultiArray()
            mod = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
            msg.data = [round(dx, 3), round(dy, 3), round(dz, 3), self.N, mod]
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 5
            msg.layout.dim[0].stride = 1
            self.publisher['mrs_data'].publish(msg)
            msg_data = Float64()
            msg_data.data = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
            self.publisher['mrs_data_mod'].publish(msg_data)

            if abs(msg_data.data) > self.ul:
                dx = (dx / msg_data.data) * self.ul
                dy = (dy / msg_data.data) * self.ul
                dz = (dz / msg_data.data) * self.ul

            if self.pose.position.z < 0.6 and dz < 0.0 and '_v' in self.controller_type:
                dz = 0.0

            if self.pose.position.z > 3.0 and dz > 0.0 and '_v' in self.controller_type:
                dz = 0.0

            mod = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dx, 2))
            msg.data = [round(dx, 3), round(dy, 3), round(dz, 3), self.N, mod]
            self.publisher['mrs_data_gt'].publish(msg)
            msg_data = Float64()
            msg_data.data = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dx, 2))
            self.publisher['mrs_data_gt_mod'].publish(msg_data)

            if '_v' in self.controller_type:
                self.target_pose.pose.position.x = self.pose.position.x
                self.target_pose.pose.position.y = self.pose.position.y
                self.target_pose.pose.position.z = self.pose.position.z
                self.target_twist.linear.x = dx
                self.target_twist.linear.y = dy
                self.target_twist.linear.z = dz
            else:
                self.target_pose.pose.position.x = self.pose.position.x + dx
                self.target_pose.pose.position.y = self.pose.position.y + dy
                self.target_pose.pose.position.z = self.pose.position.z + dz

            delta = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
            angles = tf_transformations.euler_from_quaternion(
                (self.pose.orientation.x,
                 self.pose.orientation.y,
                 self.pose.orientation.z,
                 self.pose.orientation.w))
            '''
            mean = delta/len(self.state)
            for i in range(0,len(self.state)-1):
                self.state[i] = self.state[i+1]
                mean += self.state[i]/len(self.state)

            self.state[len(self.state)-1] = delta

            if mean < 0.05 and self.update_gain:
                self.node.get_logger().info('Agent %s: Gain updated' % (self.id))
                self.update_gain = False
                for agent in self.agent_list:
                    if agent.id == 'origin':
                        agent.k = 1.0
            '''
            if delta < 0.01:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx, 2) + pow(dy, 2))
                roll = 0.0
                pitch = -atan2(dz, h)
                yaw = atan2(dy, dx)  # noqa: F841 -- ver AUDIT.md (rama doc): posible bug,
                # 0.0 en vez de yaw en la línea de abajo (pose_gradient_controller() sí usa yaw)

            q = tf_transformations.quaternion_from_euler(roll, pitch, 0.0)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)
            self.publisher_global_error_.publish(msg_error)

            self.node.get_logger().debug(
                'Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' %
                (self.pose.position.x,
                 self.target_pose.pose.position.x,
                 self.pose.position.y,
                 self.target_pose.pose.position.y,
                 self.pose.position.z,
                 self.target_pose.pose.position.z))
            if self.target_pose.pose.position.z < 0.6:
                self.target_pose.pose.position.z = 0.6

            if self.target_pose.pose.position.z > 3.0:
                self.target_pose.pose.position.z = 3.0

    def pose_gradient_controller(self):
        if self.formation:
            dx = dy = dz = 0
            for agent in self.agent_list:
                error_x = self.pose.position.x - agent.pose.position.x
                error_y = self.pose.position.y - agent.pose.position.y
                error_z = self.pose.position.z - agent.pose.position.z
                distance = pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2)
                d = sqrt(distance)
                dx += self.k * agent.k * (pow(agent.x, 2) - pow(error_x, 2)) * error_x / d
                dy += self.k * agent.k * (pow(agent.y, 2) - pow(error_y, 2)) * error_y / d
                dz += self.k * agent.k * (pow(agent.z, 2) - pow(error_z, 2)) * error_z / d

            if not self.continuous:
                delta = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
                if not self.eval_threshold(0.0, delta):
                    return

            msg = Bool()
            msg.data = True
            self.event_x.publish(msg)

            if dx > self.ul:
                dx = self.ul
            if dx < self.ll:
                dx = self.ll
            if dy > self.ul:
                dy = self.ul
            if dy < self.ll:
                dy = self.ll
            if dz > self.ul:
                dz = self.ul
            if dz < self.ll:
                dz = self.ll

            self.target_pose.pose.position.x = self.pose.position.x + dx
            self.target_pose.pose.position.y = self.pose.position.y + dy
            self.target_pose.pose.position.z = self.pose.position.z + dz

            delta = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
            angles = tf_transformations.euler_from_quaternion(
                (self.pose.orientation.x,
                 self.pose.orientation.y,
                 self.pose.orientation.z,
                 self.pose.orientation.w))

            if delta < 0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx, 2) + pow(dy, 2))
                roll = 0.0
                pitch = -atan2(dz, h)
                yaw = atan2(dy, dx)

            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

            self.node.get_logger().debug(
                'Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' %
                (self.pose.position.x,
                 self.target_pose.pose.position.x,
                 self.pose.position.y,
                 self.target_pose.pose.position.y,
                 self.pose.position.z,
                 self.target_pose.pose.position.z))
            if self.target_pose.pose.position.z < 0.6:
                self.target_pose.pose.position.z = 0.6

            if self.target_pose.pose.position.z > 2.5:
                self.target_pose.pose.position.z = 2.5

    def eval_threshold(self, signal, ref):
        # Disabled alternative noise (Cn) estimate, kept for reference:
        # mean = signal/len(self.noise)
        # for i in range(0,len(self.noise)-2):
        #     self.noise[i] = self.noise[i+1]
        #     mean += self.noise[i]/len(self.noise)
        #
        # self.noise[len(self.noise)-1] = signal
        #
        # trigger_cn = 0.0
        # for i in range(0,len(self.noise)-1):
        #     if abs(self.noise[i]-mean) > trigger_cn:
        #         trigger_cn = self.noise[i]-mean
        trigger_cn = 0.0

        # a
        a = self.trigger_ai * abs(signal - ref)
        if a > self.trigger_ai:
            a = self.trigger_ai

        # Threshold
        self.th = self.trigger_co + a + trigger_cn
        self.inc = abs(abs(ref - signal) - self.trigger_last_signal)
        # Delta Error
        if (self.inc >= abs(self.th)):
            self.trigger_last_signal = abs(ref - signal)
            return True

        return False

    def update_position_relay(self, error):
        if not self.relay_level:
            if error > self.relay_p_threshold:
                self.relay_level = True
                return self.relay_p_cmd + self.rele_cmd_eq
            else:
                return -self.relay_p_cmd + self.rele_cmd_eq
        else:
            if error < -self.relay_p_threshold:
                self.relay_level = False
                return -self.relay_p_cmd + self.rele_cmd_eq
            else:
                return self.relay_p_cmd + self.rele_cmd_eq

    def update_speed_relay(self, error):
        if not self.relay_level:
            if error > self.relay_s_threshold:
                self.relay_level = True
                return self.relay_s_cmd + self.rele_cmd_eq
            else:
                return -self.relay_s_cmd + self.rele_cmd_eq
        else:
            if error < -self.relay_s_threshold:
                self.relay_level = False
                return -self.relay_s_cmd + self.rele_cmd_eq
            else:
                return self.relay_s_cmd + self.rele_cmd_eq

    def update_attitude_relay(self, error):
        if not self.relay_level:
            if error > self.relay_a_threshold:
                self.relay_level = True
                return self.relay_a_cmd + self.rele_cmd_eq
            else:
                return -self.relay_a_cmd + self.rele_cmd_eq
        else:
            if error < -self.relay_a_threshold:
                self.relay_level = False
                return -self.relay_a_cmd + self.rele_cmd_eq
            else:
                return self.relay_a_cmd + self.rele_cmd_eq

    def update_rate_relay(self, error):
        if not self.relay_level:
            if error > self.relay_r_threshold:
                self.relay_level = True
                return self.relay_r_cmd + self.rele_cmd_eq
            else:
                return -self.relay_r_cmd + self.rele_cmd_eq
        else:
            if error < -self.relay_r_threshold:
                self.relay_level = False
                return -self.relay_r_cmd + self.rele_cmd_eq
            else:
                return self.relay_r_cmd + self.rele_cmd_eq

    ###################
    #    Iteration    #
    ###################
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        # Get measurements
        dt = self.robot.getTime() - self.past_time
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        self.global_yaw = yaw
        roll_rate = self.gyro.getValues()[0]
        pitch_rate = self.gyro.getValues()[1]
        yaw_rate = self.gyro.getValues()[2]
        x_global = self.gps.getValues()[0]  # - self.first_x_global
        y_global = self.gps.getValues()[1]  # - self.first_y_global
        z_global = self.gps.getValues()[2]
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        self.variables['pose'].position.x = x_global
        self.variables['pose'].position.y = y_global
        self.variables['pose'].position.z = z_global
        self.variables['pose'].orientation.x = q[0]
        self.variables['pose'].orientation.y = q[1]
        self.variables['pose'].orientation.z = q[2]
        self.variables['pose'].orientation.w = q[3]
        
        if not self.variables['init_pose']:
            self.variables['past_x_global'] = x_global
            self.variables['target_pose'].pose.position.x = x_global
            self.variables['past_y_global'] = y_global
            self.variables['target_pose'].pose.position.y = y_global
            self.variables['init_pose'] = True
            self.variables['last_pose'] = copy.deepcopy(self.variables['pose'])
            init_pose = PoseStamped()
            init_pose.header.frame_id = "map"
            init_pose.pose = self.variables['last_pose']
            init_pose.header.stamp = self.node.get_clock().now().to_msg()
            if not self.variables['disconnect']:
                self.publisher['pose_publisher'].publish(init_pose)
            self.z_controller.past_time = self.past_time
            self.w_controller.past_time = self.past_time
            self.take_off()

        vx_global = (x_global - self.variables['past_x_global']) / dt
        vy_global = (y_global - self.variables['past_y_global']) / dt
        vz_global = (z_global - self.variables['past_z_global']) / dt

        t_base = TransformStamped()
        t_base.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        t_base.header.frame_id = 'map'
        if self.driver_cfg['digital_twin']:
            base_name = self.id + '_dt/base_link'
        else:
            base_name = self.id + '/base_link'
        t_base.child_frame_id = base_name
        t_base.transform.translation.x = x_global
        t_base.transform.translation.y = y_global
        t_base.transform.translation.z = z_global
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tfbr.sendTransform(t_base)

        delta = np.array([x_global-self.variables['last_pose'].position.x, 
                          y_global-self.variables['last_pose'].position.y, 
                          z_global-self.variables['last_pose'].position.z])

        if (self.driver_cfg['communication'] or np.linalg.norm(delta) > 0.01) and not self.variables['disconnect']:
            PoseStamp = PoseStamped()
            PoseStamp.header.frame_id = "map"
            PoseStamp.pose = copy.deepcopy(self.variables['pose'])
            PoseStamp.header.stamp = self.node.get_clock().now().to_msg()
            if self.driver_cfg['path_enable']:
                self.variables['path'].header.stamp = self.node.get_clock().now().to_msg()
                self.variables['path'].poses.append(PoseStamp)
                self.publisher['path'].publish(self.variables['path'])
            self.publisher['pose_publisher_gt'].publish(PoseStamp)
            if 'ML2' in self.driver_cfg['controller_type']:
                PoseStamp.pose.position = self.geometry.projection(self.pose.position)
            self.publisher['pose_publisher'].publish(PoseStamp)
            self.variables['last_pose'] = copy.deepcopy(self.variables['pose'])

        # Position Controller
        # Z Controller
        if not self.variables['formation'] or '_p' in self.variables['controller_type']:
            if self.z_controller.eval_threshold(z_global, self.variables['target_pose'].pose.position.z) or self.driver_cfg['controller_protocol']:
                self.z_controller.error[0] = (self.variables['target_pose'].pose.position.z - z_global)
                dtz = self.robot.getTime() - self.z_controller.past_time
                self.variables['target_twist'].linear.z = self.z_controller.update(dtz)
                self.z_controller.past_time = self.robot.getTime()
                msg = Bool()
                msg.data = True
                self.publisher['event_z'].publish(msg)
                self.node.get_logger().debug('Z Controller Event. Th: %.4f; Inc: %.4f; dT: %.4f' %(self.z_controller.th, self.z_controller.inc, dtz))
            else:
                self.variables['target_twist'].linear.z = self.z_controller.last_value

        if self.w_controller.eval_threshold(vz_global, self.variables['target_twist'].linear.z) or self.driver_cfg['controller_protocol']:
            self.w_controller.error[0] = (self.variables['target_twist'].linear.z - vz_global)
            dtw = self.robot.getTime() - self.w_controller.past_time
            cmd_thrust = self.w_controller.update(dtw) * 1000 + 38000
            self.w_controller.past_time = self.robot.getTime()
            self.node.get_logger().debug('W Controller Event.V: %.2f; dT: %.3f' % (cmd_thrust, dtw))
        else:
            cmd_thrust = self.w_controller.last_value * 1000 + 38000

        # X-Y Controller
        if not self.variables['formation'] or '_p' in self.variables['controller_type']:
            # X Controller
            if self.x_controller.eval_threshold(x_global, self.variables['target_pose'].pose.position.x) or self.driver_cfg['controller_protocol']:
                self.x_controller.error[0] = self.variables['target_pose'].pose.position.x - x_global
                dtx = self.robot.getTime() - self.x_controller.past_time
                self.variables['target_twist'].linear.x = self.x_controller.update(dtx)
                self.x_controller.past_time = self.robot.getTime()
                msg = Bool()
                msg.data = True
                self.publisher['event_x'].publish(msg)
            else:
                self.variables['target_twist'].linear.x = self.x_controller.last_value
            # Y Controller
            if self.y_controller.eval_threshold(y_global, self.variables['target_pose'].pose.position.y) or self.driver_cfg['controller_protocol']:
                self.y_controller.error[0] = self.variables['target_pose'].pose.position.y - y_global
                dty = self.robot.getTime() - self.y_controller.past_time
                self.variables['target_twist'].linear.y = self.y_controller.update(dty)
                self.y_controller.past_time = self.robot.getTime()
                msg = Bool()
                msg.data = True
                self.publisher['event_y'].publish(msg)
            else:
                self.variables['target_twist'].linear.y = self.y_controller.last_value

        # dX-dY Controller
        if self.u_controller.eval_threshold(vx_global, self.variables['target_twist'].linear.x) or self.driver_cfg['controller_protocol']:
            self.u_controller.error[0] = (self.variables['target_twist'].linear.x - vx_global) * cos(yaw) + (self.variables['target_twist'].linear.y - vy_global) * sin(yaw)
            dtu = self.robot.getTime() - self.u_controller.past_time
            pitch_ref = self.u_controller.update(dtu)
            self.u_controller.past_time = self.robot.getTime()
        else:
            pitch_ref = self.u_controller.last_value

        if self.v_controller.eval_threshold(vy_global, self.variables['target_twist'].linear.y) or self.driver_cfg['controller_protocol']:
            self.v_controller.error[0] = -(self.variables['target_twist'].linear.x - vx_global) * sin(yaw) + (self.variables['target_twist'].linear.y - vy_global) * cos(yaw)
            dtv = self.robot.getTime() - self.v_controller.past_time
            roll_ref = self.v_controller.update(dtv)
            self.v_controller.past_time = self.robot.getTime()
        else:
            roll_ref = self.v_controller.last_value

        if self.driver_cfg['local_twist_enable']:
            msg = Twist()
            msg = self.variables['target_twist']
            self.publisher['twist'].publish(msg)

        # Attitude Controller
        # Pitch Controller
        self.pitch_controller.error[0] = pitch_ref - degrees(pitch)
        dpitch_ref = self.pitch_controller.update(dt)
        # Roll Controller
        self.roll_controller.error[0] = roll_ref - degrees(roll)
        droll_ref = self.roll_controller.update(dt)
        # Yaw Controller
        angles = tf_transformations.euler_from_quaternion((self.variables['target_pose'].pose.orientation.x, self.variables['target_pose'].pose.orientation.y, self.variables['target_pose'].pose.orientation.z, self.variables['target_pose'].pose.orientation.w))
        self.yaw_controller.error[0] = angles[2] - yaw
        if self.yaw_controller.error[0] > pi:
            self.yaw_controller.error[0] = self.yaw_controller.error[0] - 2 * pi
        if self.yaw_controller.error[0] < -pi:
            self.yaw_controller.error[0] = self.yaw_controller.error[0] + 2 * pi

        dyaw_ref = self.yaw_controller.update(dt)

        if self.driver_cfg['data_attitude_enable']:
            msg = Float64MultiArray()
            msg.data = {roll_ref, pitch_ref, angles[2], degrees(roll), degrees(pitch), yaw}
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 6
            msg.layout.dim[0].stride = 1
            self.publisher['attitude'].publish(msg)

        # Rate Controller
        self.dpitch_controller.error[0] = dpitch_ref - degrees(pitch_rate)
        delta_pitch = self.dpitch_controller.update(dt)
        self.droll_controller.error[0] = droll_ref - degrees(roll_rate)
        delta_roll = self.droll_controller.update(dt)
        self.dyaw_controller.error[0] = dyaw_ref - degrees(yaw_rate)
        delta_yaw = self.dyaw_controller.update(dt)

        if self.driver_cfg['data_rate_enable']:
            msg = Float64MultiArray()
            msg.data = {droll_ref, dpitch_ref, dyaw_ref, degrees(roll_rate), degrees(pitch_rate), degrees(yaw_rate)}
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 6
            msg.layout.dim[0].stride = 1
            self.publisher['rate'].publish(msg)

        self.node.get_logger().debug('IPC:: V: %.4f W: %.4f' % (self.variables['target_twist'].linear.x, dyaw_ref))
        if self.driver_cfg['data_enable']:
            msg = Float64MultiArray()
            msg.data = {delta_roll, delta_pitch, delta_yaw}
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 3
            msg.layout.dim[0].stride = 1
            self.publisher['data'].publish(msg)

        # Motor mixing Controller
        motorPower_m1 = (cmd_thrust - 0.5 * delta_roll - 0.5 * delta_pitch + delta_yaw)
        motorPower_m2 = (cmd_thrust - 0.5 * delta_roll + 0.5 * delta_pitch - delta_yaw)
        motorPower_m3 = (cmd_thrust + 0.5 * delta_roll + 0.5 * delta_pitch + delta_yaw)
        motorPower_m4 = (cmd_thrust + 0.5 * delta_roll - 0.5 * delta_pitch - delta_yaw)

        if self.driver_cfg['data_motor_enable']:
            msg = Float64MultiArray()
            msg.data = {cmd_thrust, motorPower_m1, motorPower_m2, motorPower_m3, motorPower_m4}
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 5
            msg.layout.dim[0].stride = 1
            self.publisher['motors'].publish(msg)

        self.node.get_logger().debug('Thrust(C): %.2f CRoll(C): %.2f CPitch(C): %.2f CYaw(C): %.2f' % (cmd_thrust, delta_roll, delta_pitch, delta_yaw))

        scaling = 1000  # TO-DO, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(-motorPower_m1 / scaling)
        self.m2_motor.setVelocity(motorPower_m2 / scaling)
        self.m3_motor.setVelocity(-motorPower_m3 / scaling)
        self.m4_motor.setVelocity(motorPower_m4 / scaling)

        self.past_time = self.robot.getTime()
        self.variables['past_x_global'] = x_global
        self.variables['past_y_global'] = y_global
        self.variables['past_z_global'] = z_global

    def IPC_controller(self):
        Vmax = 0.5
        K1 = 0.005
        Kp = 1.0
        Ki = 0.008

        d = sqrt(pow(self.target_pose.pose.position.x - self.pose.position.x, 2) +
                 pow(self.target_pose.pose.position.y - self.pose.position.y, 2)) * 100
        alpha = atan2(
            self.target_pose.pose.position.y -
            self.pose.position.y,
            self.target_pose.pose.position.x -
            self.pose.position.x)
        oc = alpha - self.global_yaw
        eo = atan2(sin(oc), cos(oc))
        p = (3.14 - abs(eo)) / 3.14
        V = min(K1 * d * p, Vmax)

        self.eomas = eo + self.eomas
        w = Kp * sin(eo) + Ki * self.eomas * 0.003

        # Cmd_Vel
        out = Twist()
        out.linear.x = V * cos(self.global_yaw)
        out.linear.y = V * sin(self.global_yaw)
        out.angular.z = w

        return out
