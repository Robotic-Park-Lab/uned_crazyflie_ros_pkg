import logging
import time
import rclpy
from threading import Timer
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import os

from rclpy.node import Node
from std_msgs.msg import String, UInt16MultiArray, Float64, Float64MultiArray
from geometry_msgs.msg import Pose, Twist, Point, TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from math import cos, sin, degrees, radians, pi, sqrt
from nav_msgs.msg import Path
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# List of URIs, comment the one you do not want to fly
uris = set()
dron = list()

class Agent():
    def __init__(self, parent, node, id, x = None, y = None, z = None, d = None):
        self.id = id
        self.idn = float(len(parent.agent_list))
        self.distance = False
        self.parent = parent
        self.node = node
        self.last_error = 0.0
        self.last_iae = 0.0
        self.k = 1.0
        self.pose = Pose()
        if d == None:
            self.x = x
            self.y = y
            self.z = z
            self.node.get_logger().info('Agent: %s' % self.str_())
        else:
            self.d = d
            self.distance = True
            self.node.get_logger().info('Agent: %d %s' % (self.idn, self.str_distance_()))
        if self.id == 'origin':
            self.pose.position.x = 0.0
            self.pose.position.y = 0.0
            self.pose.position.z = 0.8
            self.k = 2.0
        self.sub_pose_ = self.node.create_subscription(PoseStamped, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        if self.parent.config['task']['Onboard']:
            parent.scf.cf.high_level_commander.new_neighbour(self.idn, self.d, self.k)
        self.sub_d_ = self.node.create_subscription(Float64, '/' + self.id + '/d', self.d_callback, 10)
        self.publisher_data_ = self.node.create_publisher(Float64, self.parent.id + '/' + self.id + '/data', 10)
        self.publisher_error_ = self.node.create_publisher(Float64, self.parent.id + '/' + self.id + '/error', 10)
        self.publisher_iae_ = self.node.create_publisher(Float64, self.parent.id + '/' + self.id + '/iae', 10)
        self.publisher_marker_ = self.node.create_publisher(Marker, self.parent.id + '/' + self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))
    
    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def d_callback(self, msg):
        self.d = msg.data

    def gtpose_callback(self, msg):
        self.pose = msg.pose
        if self.parent.config['task']['Onboard']:
            self.parent.scf.cf.high_level_commander.update_neighbour(self.idn, self.pose.position.x, self.pose.position.y, self.pose.position.z)
        self.node.get_logger().debug('Agent: X: %.2f Y: %.2f Z: %.2f' % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

        line = Marker()
        p0 = Point()
        p0.x = self.parent.pose.position.x
        p0.y = self.parent.pose.position.y
        p0.z = self.parent.pose.position.z

        p1 = Point()
        p1.x = self.pose.position.x
        p1.y = self.pose.position.y
        p1.z = self.pose.position.z

        line.header.frame_id = 'map'
        line.header.stamp = self.node.get_clock().now().to_msg()
        line.id = 1
        line.type = 5
        line.action = 0
        line.scale.x = 0.01
        line.scale.y = 0.01
        line.scale.z = 0.01
        
        if self.distance:
            # self.parent.distance_formation_bool = False
            distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
            msg_data = Float64()
            msg_data.data = distance
            self.publisher_data_.publish(msg_data)
            if abs(distance - self.d) > 0.05:
                line.color.r = 1.0
            else:
                if abs(distance - self.d) > 0.025:
                    line.color.r = 1.0
                    line.color.g = 0.5
                else:
                    line.color.g = 1.0
        else:
            dx = p0.x-p1.x
            dy = p0.y-p1.y
            dz = p0.z-p1.z
            if abs(dx) > 0.05 or abs(dy) > 0.05 or abs(dz) > 0.05:
                line.color.r = 1.0
            else:
                if abs(dx) > 0.025 or abs(dy) > 0.025 or abs(dz) > 0.025:
                    line.color.r = 1.0
                    line.color.g = 0.5
                else:
                    line.color.g = 1.0
        line.color.a = 1.0
        line.points.append(p1)
        line.points.append(p0)

        self.publisher_marker_.publish(line)


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
    
    def rele_update(self,dt):
        if not self.rele:
            if self.error[0] > self.range:
                out = self.cmd
                self.rele = True
            else:
                out = -self.cmd
        else:
            if self.error[0] < -self.range:
                out = -self.cmd
                self.rele = False
            else: 
                out = self.cmd
        
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


class CMD_Motion():
    def __init__(self, logger, xy_lim = 10):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0
        self.thrust = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.logger = logger
        self.flight_time = 1.0
        self.xy_lim = xy_lim

    def ckeck_pose(self):
        # X Check
        if abs(self.x) > self.xy_lim*0.9:
            if abs(self.x) > self.xy_lim:
                self.logger.error('X: Error')
                if self.x > 0:
                    self.x = 0.85 * self.xy_lim
                else:
                    self.x = -0.85 * self.xy_lim
                self.logger.warning('New Point: %s' % self.pose_str_())
            else:
                self.logger.warning('X: Warning')
        # Y Check
        if abs(self.y) > self.xy_lim*0.9:
            if abs(self.y) > self.xy_lim:
                self.logger.error('Y: Error')
                if self.y > 0:
                    self.y = 0.85 * self.xy_lim
                else:
                    self.y = -0.85 * self.xy_lim
                self.logger.warning('New Point: %s' % self.pose_str_())
            else:
                self.logger.warning('Y: Warning')

    def str_(self):
        return ('Thrust: ' + str(self.thrust) + ' Roll: ' + str(self.roll) +
                ' Pitch: ' + str(self.pitch)+' Yaw: ' + str(self.yaw))

    def pose_str_(self):
        return ('X: ' + str(self.x) + ' Y: ' + str(self.y) +
                ' Z: ' + str(self.z)+' Yaw: ' + str(self.yaw))

    def send_pose_data_(self, cf, relative_pose=False):
        self.logger.debug('Goal Pose: X: %.4f Y: %.4f Z: %.4f' % (self.x, self.y, self.z))
        # cf.commander.send_position_setpoint(self.x, self.y, self.z, self.yaw)
        cf.high_level_commander.go_to(self.x, self.y, self.z, self.yaw, 1.5)

    def send_offboard_setpoint_(self, cf):
        self.logger.debug('Command: %s' % self.str_())
        cf.commander.send_setpoint(self.roll, -self.pitch, self.yaw, self.thrust)

    def take_off(self, cf):
        self.logger.info('Take off ... ')
        cf.high_level_commander.takeoff(0.75, 1.5)

    def land(self, cf):
        self.logger.info('Take land ... ')
        cf.high_level_commander.land(0.0, 2.0)


######################
## CF Logging Class ##
######################
class Crazyflie_ROS2():
    def __init__(self, parent, scf, link_uri, id, config):
        self.scf = scf
        self.parent = parent
        self.config = config
        self.id = id

        ## Intialize Variables
        self.ready = False
        self.swarm_ready = False
        self.digital_twin = self.config['type'] == 'digital_twin'
        self.target_twist = Twist()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.z = 0.0
        self._is_flying = False
        self.init_pose = False
        self.formation = False
        self.pose = Pose()
        self.home = Pose()
        self.path = Path()
        self.path.header.frame_id = "map"
        self.pitch_controller = PIDController(0.8395, 0.8483, 0.0, 0.0, 100, 30, -30, 0.1, 0.1)
        self.gimbal = False
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.sp_roll = 0.0
        self.sp_pitch = 0.0
        self.sp_yaw = 0.0
        self.scf.CONTROL_MODE = self.config['control_mode']
        self.parent.get_logger().info('CF%s::Control Mode: %s!' % (self.scf.cf.link_uri[-2:], self.scf.CONTROL_MODE))
        if self.scf.CONTROL_MODE == 'Gimbal':
            self.iterate_loop = self.parent.create_timer(0.01, self.iterate)
        self.scf.CONTROLLER_TYPE = self.config['controller']['type']
        self.parent.get_logger().info('CF%s::Controller Type: %s!' % (self.scf.cf.link_uri[-2:], self.scf.CONTROLLER_TYPE))
        self.communication = (self.config['communication']['type'] == 'Continuous')
        if not self.communication:
            self.threshold = config['communication']['threshold']['co']
        else:
            self.threshold = 0.001

        ## Intialize Crazyflie configuration
        self.scf.uri = link_uri
        self.tfbr = TransformBroadcaster(self.parent)
        self.scf.cf.connected.add_callback(self._connected)
        self.scf.cf.disconnected.add_callback(self._disconnected)
        self.scf.cf.connection_failed.add_callback(self._connection_failed)
        self.scf.cf.connection_lost.add_callback(self._connection_lost)

    def _connected(self, link_uri):
        self.parent.get_logger().info('Connected to %s -> Crazyflie %s' % (link_uri, self.scf.cf.link_uri[-2:]))
        # ROS
        # Publisher
        # POSE3D
        if self.config['local_pose']['enable']:
            self.path_enable = self.config['local_pose']['path']
            if self.path_enable:
                self.path_publisher = self.parent.create_publisher(Path, self.id + '/path', 10)
            if self.scf.CONTROL_MODE == 'None':
                self.parent.create_subscription(PoseStamped, self.id + '/pose', self.newpose_callback, 10)
                self.publisher_pose = self.parent.create_publisher(PoseStamped, self.id + '/pose', 10)
            elif self.scf.CONTROL_MODE == 'Gimbal':
                self.publisher_sp_pitch = self.parent.create_publisher(Float64, self.id + '/sp_pitch', 10)
                self.sub_goal_roll_ = self.parent.create_subscription(Float64, self.id + '/goal_roll', self.roll_callback, 10)
                self.sub_goal_pitch_ = self.parent.create_subscription(Float64, self.id + '/goal_pitch', self.pitch_callback, 10)
                self.sub_goal_yaw_ = self.parent.create_subscription(Float64, self.id + '/goal_yaw', self.yaw_callback, 10)
            else:
                self.publisher_pose = self.parent.create_publisher(PoseStamped, self.id + '/local_pose', 10)
            if self.digital_twin:
                self.publisher_dtpose = self.parent.create_publisher(PoseStamped, self.id + '/pose_dt', 10)
            self.publisher_roll = self.parent.create_publisher(Float64, self.id + '/roll', 10)
            self.publisher_pitch = self.parent.create_publisher(Float64, self.id + '/pitch', 10)
            self.publisher_yaw = self.parent.create_publisher(Float64, self.id + '/yaw', 10)
            self._lg_stab_pose = LogConfig(name='Pose', period_in_ms=self.config['local_pose']['T'])
            self._lg_stab_pose.add_variable('stateEstimate.x', 'float')
            self._lg_stab_pose.add_variable('stateEstimate.y', 'float')
            self._lg_stab_pose.add_variable('stateEstimate.z', 'float')
            self._lg_stab_pose.add_variable('stabilizer.roll', 'float')
            self._lg_stab_pose.add_variable('stabilizer.pitch', 'float')
            self._lg_stab_pose.add_variable('stabilizer.yaw', 'float')
            try:
                self.scf.cf.log.add_config(self._lg_stab_pose)
                self._lg_stab_pose.data_received_cb.add_callback(self._stab_log_data)
                self._lg_stab_pose.error_cb.add_callback(self._stab_log_error)
                self._lg_stab_pose.start()
            except KeyError as e:
                self.parent.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])
        # TWIST
        if self.config['local_twist']['enable']:
            self.publisher_twist = self.parent.create_publisher(Twist, self.id + '/local_twist', 10)
            self._lg_stab_twist = LogConfig(name='Twist', period_in_ms=self.config['local_twist']['T'])
            self._lg_stab_twist.add_variable('gyro.x', 'float')
            self._lg_stab_twist.add_variable('gyro.y', 'float')
            self._lg_stab_twist.add_variable('gyro.z', 'float')
            self._lg_stab_twist.add_variable('stateEstimate.vx', 'float')
            self._lg_stab_twist.add_variable('stateEstimate.vy', 'float')
            self._lg_stab_twist.add_variable('stateEstimate.vz', 'float')
            try:
                self.scf.cf.log.add_config(self._lg_stab_twist)
                self._lg_stab_twist.data_received_cb.add_callback(self._stab_log_data)
                self._lg_stab_twist.error_cb.add_callback(self._stab_log_error)

                self._lg_stab_twist.start()
            except KeyError as e:
                self.parent.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # DATA ATTITUDE.
        if self.config['data_attitude']['enable']:
            self.publisher_data_attitude = self.parent.create_publisher(Float64MultiArray, self.id + '/data_attitude', 10)
            self._lg_stab_data_a = LogConfig(name='Data_attitude', period_in_ms=self.config['data_attitude']['T'])
            self._lg_stab_data_a.add_variable('posCtl.targetVX', 'float')
            self._lg_stab_data_a.add_variable('posCtl.targetVY', 'float')
            self._lg_stab_data_a.add_variable('controller.roll', 'float')
            self._lg_stab_data_a.add_variable('controller.pitch', 'float')
            self._lg_stab_data_a.add_variable('controller.yaw', 'float')
            try:
                self.scf.cf.log.add_config(self._lg_stab_data_a)
                self._lg_stab_data_a.data_received_cb.add_callback(self._stab_log_data)
                self._lg_stab_data_a.error_cb.add_callback(self._stab_log_error)

                self._lg_stab_data_a.start()
            except KeyError as e:
                self.parent.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # DATA RATE.
        if self.config['data_rate']['enable']:
            self.publisher_data_rate = self.parent.create_publisher(Float64MultiArray, self.id + '/data_rate', 10)
            self._lg_stab_data_r = LogConfig(name='Data_rate', period_in_ms=self.config['data_rate']['T'])
            self._lg_stab_data_r.add_variable('controller.rollRate', 'float')
            self._lg_stab_data_r.add_variable('controller.pitchRate', 'float')
            self._lg_stab_data_r.add_variable('controller.yawRate', 'float')
            self._lg_stab_data_r.add_variable('controller.cmd_roll', 'float')
            self._lg_stab_data_r.add_variable('controller.cmd_pitch', 'float')
            self._lg_stab_data_r.add_variable('controller.cmd_yaw', 'float')
            try:
                self.scf.cf.log.add_config(self._lg_stab_data_r)
                self._lg_stab_data_r.data_received_cb.add_callback(self._stab_log_data)
                self._lg_stab_data_r.error_cb.add_callback(self._stab_log_error)

                self._lg_stab_data_r.start()
            except KeyError as e:
                self.parent.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # DATA MOTOR.
        if self.config['data_motor']['enable']:
            self.publisher_data_motor = self.parent.create_publisher(Float64MultiArray, self.id + '/data_motor', 10)
            self._lg_stab_data_m = LogConfig(name='Data_motor', period_in_ms=self.config['data_rate']['T'])
            self._lg_stab_data_m.add_variable('posCtl.targetVZ', 'float')
            self._lg_stab_data_m.add_variable('controller.cmd_thrust', 'float')
            self._lg_stab_data_m.add_variable('motor.m1', 'float')
            self._lg_stab_data_m.add_variable('motor.m2', 'float')
            self._lg_stab_data_m.add_variable('motor.m3', 'float')
            self._lg_stab_data_m.add_variable('motor.m4', 'float')
            try:
                self.scf.cf.log.add_config(self._lg_stab_data_m)
                self._lg_stab_data_m.data_received_cb.add_callback(self._stab_log_data)
                self._lg_stab_data_m.error_cb.add_callback(self._stab_log_error)

                self._lg_stab_data_m.start()
            except KeyError as e:
                self.parent.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # MULTIROBOT
         # DATA.
        if True:
            self.publisher_mrs_data = self.parent.create_publisher(Float64MultiArray, self.id + '/mr_data', 10)
            self._lg_stab_data = LogConfig(name='Data_multirobot', period_in_ms=100)
            self._lg_stab_data.add_variable('multirobot.cmd_x', 'float')
            self._lg_stab_data.add_variable('multirobot.cmd_y', 'float')
            self._lg_stab_data.add_variable('multirobot.cmd_z', 'float')
            self._lg_stab_data.add_variable('multirobot.n', 'uint16_t')
            try:
                self.scf.cf.log.add_config(self._lg_stab_data)
                self._lg_stab_data.data_received_cb.add_callback(self._stab_log_data)
                self._lg_stab_data.error_cb.add_callback(self._stab_log_error)

                self._lg_stab_data.start()
            except KeyError as e:
                self.parent.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])
        
        # DATA.
        if self.config['data']['enable']:
            self.publisher_data = self.parent.create_publisher(UInt16MultiArray, self.id + '/data', 10)
            self._lg_stab_data = LogConfig(name='Data', period_in_ms=self.config['data']['T'])
            self._lg_stab_data.add_variable('posEbCtl.Zcount', 'uint16_t')
            self._lg_stab_data.add_variable('posEbCtl.Ycount', 'uint16_t')
            self._lg_stab_data.add_variable('posEbCtl.Xcount', 'uint16_t')
            self._lg_stab_data.add_variable('pm.vbat', 'FP16')
            try:
                self.scf.cf.log.add_config(self._lg_stab_data)
                self._lg_stab_data.data_received_cb.add_callback(self._stab_log_data)
                self._lg_stab_data.error_cb.add_callback(self._stab_log_error)

                self._lg_stab_data.start()
            except KeyError as e:
                self.parent.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # Subscription
        self.parent.create_subscription(String, self.id + '/order', self.order_callback, 10)
        self.parent.create_subscription(String, '/swarm/status', self.swarm_status_callback, 10)
        self.parent.create_subscription(PoseStamped, self.id + '/target_pose', self.targetpose_callback, 10)
        # self.parent.create_subscription(Time, '/swarm/time', self.time_callback, 1)
        if self.scf.CONTROL_MODE == 'HighLevel':
            self.sub_goalpose = self.parent.create_subscription(PoseStamped, self.id + '/goal_pose', self.goalpose_callback, 10)
        else:
            self.parent.create_subscription(Float64MultiArray, self.id + '/onboard_cmd', self.cmd_control_callback, 10)
        # self.parent.create_subscription(Pidcontroller, self.id + '/controllers_params', self.controllers_params_callback, 10)
        
        # Params
        '''
        self.scf.cf.param.add_update_callback(group='posCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='velCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='posEbCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='velEbCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='pid_attitude', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='pid_rate', cb=self.param_stab_est_callback)
        '''
        # self.scf.cf.param.add_update_callback(group='deck', cb=self.param_stab_est_callback)
        # self.scf.cf.param.add_update_callback(group='controller', cb=self.param_stab_est_callback)
        # self.scf.cf.param.add_update_callback(group='flightmode', cb=self.param_stab_est_callback)
        # self.scf.cf.param.add_update_callback(group='pid_attitude', cb=self.param_stab_est_callback)
        # self.scf.cf.param.add_update_callback(group='commander', cb=self.param_stab_est_callback)
        # self.scf.cf.param.add_update_callback(group='stabilizer', cb=self.param_stab_est_callback)

        
        self.xy_lim = 2.0
        self.cmd_motion_ = CMD_Motion(self.parent.get_logger(), xy_lim = self.xy_lim)
        self.scf.cf.commander.set_client_xmode(True)

    def _stab_log_error(self, logconf, msg):
        self.parent.get_logger().error('Crazyflie %s. Error when logging %s: %s' % (self.scf.cf.link_uri[-2:], logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        if(logconf.name == "Pose"):
            self.pose_callback(data)
            # print('[%d]CF%s[%s]: %s' % (timestamp, self.scf.cf.link_uri[-2:], logconf.name, data))
        elif(logconf.name == "Twist"):
            self.twist_callback(data)
        elif(logconf.name == "Data_attitude"):
            self.dataAttitude_callback(data)
        elif(logconf.name == "Data_rate"):
            self.dataRate_callback(data)
        elif(logconf.name == "Data_multirobot"):
            self.dataMRS_callback(data)
            # print('[%d]CF%s[%s]: %s' % (timestamp, self.scf.cf.link_uri[-2:], logconf.name, data))
        elif(logconf.name == "Data_motor"):
            self.dataMotor_callback(data)
            # print('[%d]CF%s[%s]: %s' % (timestamp, self.scf.cf.link_uri[-2:], logconf.name, data))
        elif(logconf.name == "Data"):
            self.data_callback(data)
        else:
            self.parent.get_logger().error('CF%s. Error: %s: not valid logconf' % (self.scf.cf.link_uri[-2:], logconf.name))

    def param_stab_est_callback(self, name, value):
        self.parent.get_logger().info('CF%s. Parameter %s: %s' %(self.scf.cf.link_uri[-2:], name, value))

    def _connection_failed(self, link_uri, msg):
        self.parent.get_logger().error('Crazyflie %s. Connection to %s failed: %s' % (self.scf.cf.link_uri[-2:], link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        self.parent.get_logger().error('Crazyflie %s. Connection to %s lost: %s' % (self.scf.cf.link_uri[-2:], link_uri, msg))

    def _disconnected(self, link_uri):
        self.parent.get_logger().warning('Crazyflie %s. Disconnected from %s' % (self.scf.cf.link_uri[-2:], link_uri))
        self.is_connected = False
        self.parent.destroy_node()

    def roll_callback(self, msg):
        self.sp_roll = msg.data

    def pitch_callback(self, msg):
        self.sp_pitch = msg.data

    def yaw_callback(self, msg):
        self.sp_yaw = msg.data

    def iterate(self):
        msg = Float64()
        self.parent.get_logger().info('CF:::SP_Pitch: %.2f, Pitch: %.2f' % (self.cmd_motion_.pitch, self.pitch))
        if self.gimbal:
            # self.parent.get_logger().info('SetPoint:::Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Thrust: %d' % (self.cmd_motion_.roll, self.cmd_motion_.pitch, self.cmd_motion_.yaw, self.cmd_motion_.thrust))
            self.pitch_controller.error[0] = self.sp_pitch - self.pitch
            # self.cmd_motion_.pitch = self.pitch_controller.update(0.01)
            self.cmd_motion_.pitch = self.sp_pitch
            # self.cmd_motion_.pitch = self.pitch_controller.rele_update(0.01)
            self.scf.cf.extpos.send_extpos(0.0, 0.0, 0.7)
            try:
                msg.data = self.cmd_motion_.pitch
                self.publisher_sp_pitch.publish(msg)
            except:
                pass
            self.cmd_motion_.send_offboard_setpoint_(self.scf.cf)

    def take_off(self):
        self.parent.get_logger().info('CF%s::Take Off.' % self.scf.cf.link_uri[-2:])
        # self.cmd_motion_.take_off(self.scf.cf)
        # self.cmd_motion_.z = 0.75
        # self.cmd_motion_.send_pose_data_(self.scf.cf)
        self.scf.cf.high_level_commander.takeoff(0.75, 2.0)
        self._is_flying = True
        self.t_ready = Timer(2, self._ready)
        self.t_ready.start()

    def _ready(self):
        self.parent.get_logger().info('CF%s::Ready!!.' % self.scf.cf.link_uri[-2:])
        self.ready = True

    def gohome(self):
        self.parent.get_logger().info('CF%s::Go Home.' % self.scf.cf.link_uri[-2:])
        self.scf.cf.high_level_commander.go_to_target_pose(self.home.position.x, self.home.position.y, 0.7)

    def descent(self):
        self.parent.get_logger().info('CF%s::Descent.' % self.scf.cf.link_uri[-2:])
        # self.cmd_motion_.z = 0.07
        # self.cmd_motion_.land(self.scf.cf)
        # self.cmd_motion_.send_pose_data_(self.scf.cf)
        self.scf.cf.high_level_commander.land(0.0, 2.0)
        self.t_desc = Timer(3, self.take_land)
        self._is_flying = False
        self.ready = False
        self.t_desc.start()

    def take_land(self):
        self.parent.get_logger().info('CF%s::Take Land.' % self.scf.cf.link_uri[-2:])
        self.scf.cf.commander.send_setpoint(0.0, 0.0, 0, 0)
        self.scf.cf.commander.send_stop_setpoint()
        self.init_pose = False

    def pose_callback(self, data):
        if self.init_pose:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.parent.get_clock().now().to_msg()
            msg.pose.position.x = data['stateEstimate.x']
            msg.pose.position.y = data['stateEstimate.y']
            msg.pose.position.z = data['stateEstimate.z']
            self.roll = data['stabilizer.roll']
            self.pitch = data['stabilizer.pitch']
            self.yaw = data['stabilizer.yaw']
            q = quaternion_from_euler(data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw'])
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]

            delta = np.array([self.pose.position.x-msg.pose.position.x,self.pose.position.y-msg.pose.position.y,self.pose.position.z-msg.pose.position.z])
            
            if self.communication or np.linalg.norm(delta)>self.threshold:
                self.pose = msg.pose
                self.publisher_pose.publish(msg)
                value = Float64()
                value.data = self.roll
                self.publisher_roll.publish(value)
                value.data = self.pitch
                self.publisher_pitch.publish(value)
                value.data = self.yaw
                self.publisher_yaw.publish(value)
                if self.digital_twin:
                    self.publisher_dtpose.publish(msg)
                t_base = TransformStamped()
                t_base.header.stamp = self.parent.get_clock().now().to_msg()
                t_base.header.frame_id = 'map'
                t_base.child_frame_id = self.id+'/base_link'
                t_base.transform.translation.x = msg.pose.position.x
                t_base.transform.translation.y = msg.pose.position.y
                t_base.transform.translation.z = msg.pose.position.z
                t_base.transform.rotation.x = msg.pose.orientation.x
                t_base.transform.rotation.y = msg.pose.orientation.y
                t_base.transform.rotation.z = msg.pose.orientation.z
                t_base.transform.rotation.w = msg.pose.orientation.w
                self.tfbr.sendTransform(t_base)
                if self.path_enable:
                    self.path.header.stamp = self.parent.get_clock().now().to_msg()
                    PoseStamp = PoseStamped()
                    PoseStamp.header.frame_id = "map"
                    PoseStamp.pose.position.x = msg.pose.position.x
                    PoseStamp.pose.position.y = msg.pose.position.y
                    PoseStamp.pose.position.z = msg.pose.position.z
                    PoseStamp.pose.orientation.x = msg.pose.orientation.x
                    PoseStamp.pose.orientation.y = msg.pose.orientation.y
                    PoseStamp.pose.orientation.z = msg.pose.orientation.z
                    PoseStamp.pose.orientation.w = msg.pose.orientation.w
                    PoseStamp.header.stamp = self.parent.get_clock().now().to_msg()
                    self.path.poses.append(PoseStamp)
                    self.path_publisher.publish(self.path)
        else:
            try:
                if self.scf.cf.param.get_value('deck.bcLighthouse4') == '1' or self.config['positioning'] == 'Intern':
                    msg = PoseStamped()
                    msg.header.frame_id = "map"
                    msg.header.stamp = self.parent.get_clock().now().to_msg()
                    msg.pose.position.x = data['stateEstimate.x']
                    msg.pose.position.y = data['stateEstimate.y']
                    msg.pose.position.z = data['stateEstimate.z']
                    self.roll = data['stabilizer.roll']
                    self.pitch = data['stabilizer.pitch']
                    self.yaw = data['stabilizer.yaw']
                    q = quaternion_from_euler(data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw'])
                    msg.pose.orientation.x = q[0]
                    msg.pose.orientation.y = q[1]
                    msg.pose.orientation.z = q[2]
                    msg.pose.orientation.w = q[3]
                    t_base = TransformStamped()
                    t_base.header.stamp = self.parent.get_clock().now().to_msg()
                    t_base.header.frame_id = 'map'
                    t_base.child_frame_id = self.id+'/base_link'
                    t_base.transform.translation.x = msg.pose.position.x
                    t_base.transform.translation.y = msg.pose.position.y
                    t_base.transform.translation.z = msg.pose.position.z
                    t_base.transform.rotation.x = msg.pose.orientation.x
                    t_base.transform.rotation.y = msg.pose.orientation.y
                    t_base.transform.rotation.z = msg.pose.orientation.z
                    t_base.transform.rotation.w = msg.pose.orientation.w
                    self.tfbr.sendTransform(t_base)
                    self.init_pose = True
                    self.pose = msg.pose
                    self.home = msg.pose
                    self.publisher_pose.publish(msg)
                    value = Float64()
                    value.data = self.roll
                    self.publisher_roll.publish(value)
                    value.data = self.pitch
                    self.publisher_pitch.publish(value)
                    value.data = self.yaw
                    self.publisher_yaw.publish(value)
                    self.cmd_motion_.x = msg.pose.position.x
                    self.cmd_motion_.y = msg.pose.position.y
                    self.cmd_motion_.z = msg.pose.position.z
                    self.parent.get_logger().info('CF%s::Home pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
            except:
                pass        

    def twist_callback(self, data):
        msg = Twist()
        msg.linear.x = data['stateEstimate.vx']
        msg.linear.y = data['stateEstimate.vy']
        msg.linear.z = data['stateEstimate.vz']
        msg.angular.x = data['gyro.x']
        msg.angular.y = data['gyro.y']
        msg.angular.z = data['gyro.z']

        self.publisher_twist.publish(msg)
    
    def dataAttitude_callback(self, data):
        msg = Float64MultiArray()
        msg.data = {data['posCtl.targetVX'], data['posCtl.targetVY'], data['controller.roll'], data['controller.pitch'], data['controller.yaw']}
        self.publisher_data_attitude.publish(msg)

    def dataRate_callback(self, data):
        msg = Float64MultiArray()
        msg.data = {data['controller.rollRate'], data['controller.pitchRate'], data['controller.yawRate'], data['controller.cmd_roll'], data['controller.cmd_pitch'], data['controller.cmd_yaw']}
        self.publisher_data_rate.publish(msg)

    def dataMRS_callback(self, data):
        msg = Float64MultiArray()
        msg.data = {data['multirobot.cmd_x'], data['multirobot.cmd_y'], data['multirobot.cmd_z'], float(data['multirobot.n'])}
        self.publisher_mrs_data.publish(msg)

    def dataMotor_callback(self, data):
        msg = Float64MultiArray()
        msg.data = {data['posCtl.targetVZ'], data['controller.cmd_thrust'], data['motor.m1'], data['motor.m2'], data['motor.m3'], data['motor.m4']}
        self.publisher_data_motor.publish(msg)

    def data_callback(self, data):
        msg = UInt16MultiArray()
        msg.data = {data['posEbCtl.Xcount'], data['posEbCtl.Ycount'], data['posEbCtl.Zcount']}
        self.publisher_data.publish(msg)

    def order_callback(self, msg):
        self.parent.get_logger().info('CF%s::Order: "%s"' % (self.scf.cf.link_uri[-2:], msg.data))
        if msg.data == 'take_off':
            if self._is_flying:
                self.parent.get_logger().warning('CF%s::Already flying' % self.scf.cf.link_uri[-2:])
            else:
                self.take_off()
        elif msg.data == 'land':
            if self._is_flying:
                self.descent()
            else:
                self.parent.get_logger().warning('CF%s::In land' % self.scf.cf.link_uri[-2:])
        elif msg.data == 'gohome':
            if self._is_flying:
                self.gohome()
            else:
                self.parent.get_logger().warning('CF%s::In land' % self.scf.cf.link_uri[-2:])
        elif msg.data == 'formation_run':
            if self.config['task']['enable']:
                self.formation = True
                self.scf.cf.high_level_commander.enable_formation()
        elif msg.data == 'formation_stop':
            self.formation = False
            self.scf.cf.param.set_value('stabilizer.controller', '1')
            self.descent()
        elif msg.data == 'gimbal':
            self.cmd_motion_.roll = 0.0
            self.cmd_motion_.pitch = 0.0
            self.cmd_motion_.yaw = 0.0
            self.gimbal = True
  
            self.scf.cf.param.set_value('stabilizer.estimator', '1')
            self.scf.cf.param.set_value('kalman.resetEstimation', '1')
            self.scf.cf.param.set_value('kalman.resetEstimation', '0')
            
            self.scf.cf.param.set_value('flightmode.stabModeRoll', '1')
            self.scf.cf.param.set_value('pid_attitude.roll_kp', 0.0)
            self.scf.cf.param.set_value('pid_attitude.roll_ki', 0.0)
            self.scf.cf.param.set_value('pid_attitude.roll_kd', 0.0)

            self.scf.cf.param.set_value('flightmode.stabModePitch', '1')
            self.scf.cf.param.set_value('pid_attitude.pitch_kp', 0.8395)
            self.scf.cf.param.set_value('pid_attitude.pitch_ki', 0.8483)
            self.scf.cf.param.set_value('pid_attitude.pitch_kd', 0.0)

            self.scf.cf.param.set_value('flightmode.stabModeYaw', '1')
            self.scf.cf.param.set_value('pid_attitude.yaw_kp', 0.0)
            self.scf.cf.param.set_value('pid_attitude.yaw_ki', 0.0)
            self.scf.cf.param.set_value('pid_attitude.yaw_kd', 0.0)

            self.scf.cf.commander.send_setpoint(0, 0, 0, 0)

            self.cmd_motion_.thrust = 1001
        else:
            self.parent.get_logger().error('CF%s::"%s": Unknown order' % (self.scf.cf.link_uri[-2:], msg.data))

    def cmd_control_callback(self, msg):
        if self.scf.CONTROL_MODE == 'OffBoard' or self.scf.CONTROL_MODE == 'Gimbal':
            self.cmd_motion_.roll = msg.data[1]
            self.cmd_motion_.pitch = msg.data[2]
            self.cmd_motion_.yaw = msg.data[3]
            self.cmd_motion_.thrust = int(msg.data[0])
            self.parent.get_logger().debug('CF%s::Command: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.str_()))
        else:
            self.parent.get_logger().warning('CF%s::New command control order. Offboard control disabled' % self.scf.cf.link_uri[-2:])

    def controllers_params_callback(self, msg):
        self.parent.get_logger().info('CF%s: New %s controller parameters' % (self.scf.cf.link_uri[-2:], msg.id))
        if (self.scf.CONTROLLER_TYPE == 'PID_Continuous'):
            if msg.id == 'x':
                groupstr = 'posCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'yVelMax', msg.upperlimit)
            elif msg.id == 'y':
                groupstr = 'posCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf.cf.param.set_value(groupstr + '.x' + msg.id + 'VelMax', msg.upperlimit)
            elif msg.id == 'z':
                groupstr = 'posCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'VelMax', msg.upperlimit)
            elif msg.id == 'vx':
                groupstr = 'velCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
            elif msg.id == 'vy':
                groupstr = 'velCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
            elif msg.id == 'vz':
                groupstr = 'velCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
            elif msg.id == 'roll':
                groupstr = 'pid_attitude'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'pitch':
                groupstr = 'pid_attitude'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'yaw':
                groupstr = 'pid_attitude'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'droll':
                groupstr = 'pid_rate'
                self.scf.cf.param.set_value(groupstr + '.' + 'roll_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + 'roll_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + 'roll_kd', msg.kd)
            elif msg.id == 'dpitch':
                groupstr = 'pid_rate'
                self.scf.cf.param.set_value(groupstr + '.' + 'pitch_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + 'pitch_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + 'pitch_kd', msg.kd)
            elif msg.id == 'dyaw':
                groupstr = 'pid_rate'
                self.scf.cf.param.set_value(groupstr + '.' + 'yaw_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + 'yaw_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + 'yaw_kd', msg.kd)
            self.get_logger().info('Kp: %0.2f \t Ki: %0.2f \t Kd: %0.2f \t N: %0.2f \t UL: %0.2f \t LL: %0.2f' % (msg.kp, msg.ki, msg.kd, msg.nd, msg.upperlimit, msg.lowerlimit))
        elif (self.scf.CONTROLLER_TYPE == 'PID_EventBased'):
            if msg.id == 'x':
                groupstr = 'posEbCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'yVelMax', msg.upperlimit)
            elif msg.id == 'y':
                groupstr = 'posEbCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
                self.scf.cf.param.set_value(groupstr + '.x' + msg.id + 'VelMax', msg.upperlimit)
            elif msg.id == 'z':
                groupstr = 'posEbCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'VelMax', msg.upperlimit)
            elif msg.id == 'vx':
                groupstr = 'velEbCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
            elif msg.id == 'vy':
                groupstr = 'velEbCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
            elif msg.id == 'vz':
                groupstr = 'velEbCtlPid'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
            elif msg.id == 'roll':
                groupstr = 'pid_attitude'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'pitch':
                groupstr = 'pid_attitude'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'yaw':
                groupstr = 'pid_attitude'
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'droll':
                groupstr = 'pid_rate'
                self.scf.cf.param.set_value(groupstr + '.' + 'roll_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + 'roll_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + 'roll_kd', msg.kd)
            elif msg.id == 'dpitch':
                groupstr = 'pid_rate'
                self.scf.cf.param.set_value(groupstr + '.' + 'pitch_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + 'pitch_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + 'pitch_kd', msg.kd)
            elif msg.id == 'dyaw':
                groupstr = 'pid_rate'
                self.scf.cf.param.set_value(groupstr + '.' + 'yaw_kp', msg.kp)
                self.scf.cf.param.set_value(groupstr + '.' + 'yaw_ki', msg.ki)
                self.scf.cf.param.set_value(groupstr + '.' + 'yaw_kd', msg.kd)
            self.get_logger().info('Kp: %0.2f \t Ki: %0.2f \t Kd: %0.2f \t N: %0.2f \t UL: %0.2f \t LL: %0.2f' % (msg.kp, msg.ki, msg.kd, msg.nd, msg.upperlimit, msg.lowerlimit))

    def swarm_status_callback(self, msg):
        self.swarm_ready = True

    def newpose_callback(self, msg):
        # self.parent.get_logger().info('CF%s::New pose: X:%.2f Y:%.2f Z:%.2f' % (self.scf.cf.link_uri[-2:], msg.position.x, msg.position.y, msg.position.z))
        if not self.init_pose:
            self.pose = msg.pose
            self.home = msg.pose
            self.publisher_pose.publish(msg)
            self.scf.cf.extpos.send_extpos(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            self.init_pose = True
            self.cmd_motion_.x = msg.pose.position.x
            self.cmd_motion_.y = msg.pose.position.y
            self.cmd_motion_.z = msg.pose.position.z
            self.parent.get_logger().info('CF%s::Init pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
        x = np.array([self.pose.position.x-msg.pose.position.x,self.pose.position.y-msg.pose.position.y,self.pose.position.z-msg.pose.position.z])
        # if (np.linalg.norm(x)>0.005 and np.linalg.norm(x)<0.2):
        self.scf.cf.extpos.send_extpos(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if ((abs(msg.pose.position.x)>self.xy_lim) or (abs(msg.pose.position.y)>self.xy_lim) or (abs(msg.pose.position.z)>2.0)) and self.scf.CONTROL_MODE != 'OffBoard':
            self.scf.CONTROL_MODE = 'HighLevel'
            self._is_flying = True
            self.parent.get_logger().error('CF%s::Out.' % self.scf.cf.link_uri[-2:])
            self.gohome()
            t_end = Timer(3, self.descent)
            t_end.start()

    def goalpose_callback(self, msg):
        if self.scf.CONTROL_MODE == 'HighLevel' and not self.formation:
            # self.cmd_motion_.x = msg.position.x
            # self.cmd_motion_.y = msg.position.y
            # self.cmd_motion_.z = msg.position.z
            # self.cmd_motion_.ckeck_pose()
            # self.cmd_motion_.send_pose_data_(self.scf.cf)
            # self.parent.get_logger().debug('CF%s::New Goal pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_())) 
            # self.parent.get_logger().debug('CF%s::New Target pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
            self.scf.cf.high_level_commander.go_to_target_pose(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def targetpose_callback(self, msg):
        self.cmd_motion_.x = msg.pose.position.x
        self.cmd_motion_.y = msg.pose.position.y
        self.cmd_motion_.z = msg.pose.position.z
        self.parent.get_logger().debug('CF%s::New Target pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
        self.scf.cf.high_level_commander.go_to_target_pose(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def time_callback(self,msg):
        time = self.parent.get_clock().now().to_msg()
        self.parent.get_logger().info('CF%s::Delay: %.2f, %.2f' % (self.scf.cf.link_uri[-2:], msg.sec - time.sec, msg.nanosec - time.nanosec))
    
    ###############
    #    Tasks    #
    ###############
    def load_formation_params(self):
        if self.config['task']['enable']:
            self.parent.destroy_subscription(self.sub_goalpose)
            self.publisher_goalpose = self.parent.create_publisher(PoseStamped, self.name_value + '/goal_pose', 10)
            self.publisher_global_error_ = self.parent.create_publisher(Float64, self.id + '/global_error', 10)
            self.parent.get_logger().info('Task %s by %s' % (self.config['task']['type'], self.config['task']['role']))
            self.agent_list = list()
            self.controller = self.config['task']['controller']
            self.controller_type = self.controller['type']
            self.k = self.controller['gain']
            self.ul = self.controller['upperLimit']
            self.ll = self.controller['lowerLimit']
            self.continuous = self.controller['protocol'] == 'Continuous'
            aux = self.config['task']['relationship']
            self.relationship = aux.split(', ')
            if self.config['task']['type'] == 'distance':
                if self.config['task']['Onboard']:
                    self.timer_task = self.parent.create_timer(self.config['task']['T']/1000, self.task_formation_info)
                else:
                    self.task_period = self.config['task']['T']/1000
                    self.timer_task = self.parent.create_timer(self.task_period, self.task_formation_distance)
                
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, self.parent, aux[0], d = float(aux[1]))
                    self.agent_list.append(robot)
            elif self.config['task']['type'] == 'relative_pose':
                self.timer_task = self.parent.create_timer(self.config['task']['T']/1000, self.task_formation_pose)
                for rel in self.relationship:
                    aux = rel.split('_')
                    rel_pose = aux[1].split('/')
                    robot = Agent(self, self.parent, aux[0], x = float(rel_pose[0]), y = float(rel_pose[1]), z = float(rel_pose[2]))
                    self.agent_list.append(robot)

    def task_formation_distance(self):
        if self.formation:
            msg_error = Float64()
            msg_error.data = 0.0
            dx = dy = dz = 0
            target_pose = Pose()
            for agent in self.agent_list:
                error_x = self.pose.position.x - agent.pose.position.x
                error_y = self.pose.position.y - agent.pose.position.y
                error_z = self.pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                dx += self.k * agent.k * (pow(agent.d,2) - distance) * error_x
                dy += self.k * agent.k * (pow(agent.d,2) - distance) * error_y
                dz += self.k * agent.k * (pow(agent.d,2) - distance) * error_z
                
                msg_data = Float64()
                msg_data.data = sqrt(distance)
                agent.publisher_data_.publish(msg_data)
                error = abs(msg_data.data - agent.d)
                msg_data.data = agent.last_iae + (agent.last_error + error) * self.task_period /2
                agent.last_error = error
                agent.publisher_iae_.publish(msg_data)
                agent.last_iae = msg_data.data
                msg_data.data = distance - pow(agent.d,2)
                agent.publisher_error_.publish(msg_data)
                msg_error.data += abs(agent.d - distance)

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
            
            target_pose.position.x = self.pose.position.x + dx
            target_pose.position.y = self.pose.position.y + dy
            target_pose.position.z = self.pose.position.z + dz
            
            # self.parent.get_logger().debug('Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (self.pose.position.x, target_pose.position.x, self.pose.position.y, target_pose.position.y, self.pose.position.z, target_pose.position.z)) 
            if target_pose.position.z < 0.5:
                target_pose.position.z = 0.5

            if target_pose.position.z > 2.0:
                target_pose.position.z = 2.0
            
            self.goalpose_callback(target_pose)
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose = target_pose
            self.publisher_goalpose.publish(msg)
            self.publisher_global_error_.publish(msg_error)

    def task_formation_info(self):
        if self.formation:
            msg_error = Float64()
            msg_error.data = 0.0
            target_pose = Pose()
            for agent in self.agent_list:
                error_x = self.pose.position.x - agent.pose.position.x
                error_y = self.pose.position.y - agent.pose.position.y
                error_z = self.pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2) 
                msg_data = Float64()
                msg_data.data = sqrt(distance)
                agent.publisher_data_.publish(msg_data)
                error = abs(msg_data.data - agent.d)
                msg_data.data = agent.last_iae + (agent.last_error + error) * self.task_period /2
                agent.last_error = error
                agent.publisher_iae_.publish(msg_data)
                agent.last_iae = msg_data.data
                msg_data.data = distance - pow(agent.d,2)
                agent.publisher_error_.publish(msg_data)
                msg_error.data += abs(agent.d - distance)
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose = target_pose
            self.publisher_goalpose.publish(msg)
            self.publisher_global_error_.publish(msg_error)


    def task_formation_pose(self):
        if self.ready and self.swarm_ready:
            # TO-DO
            dx = dy = dz = 0

#####################
## CF Swarm Class  ##
#####################
class CFSwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('config', 'file_path.yaml')
        self.declare_parameter('robots', '')

        # Publisher
        self.publisher_status_ = self.create_publisher(String,'/swarm/status', 10)
        self.publisher_order = self.create_publisher(String, '/swarm/order', 10)

        # Subscription
        self.sub_order = self.create_subscription(String, '/swarm/order', self.order_callback, 10)
        self.sub_pose_ = self.create_subscription(PoseStamped, '/swarm/pose', self.newpose_callback, 10)
        self.sub_goal_pose_ = self.create_subscription(Pose, '/swarm/goal_pose', self.goalpose_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('SwarmDriver::inicialize() ok.')

        self.pose = Pose()
        # Read Params
        aux = self.get_parameter('robots').get_parameter_value().string_value
        robot_list = aux.split(', ')
        n_robot = len(robot_list)
        config_file = self.get_parameter('config').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
            
        # Define crazyflie URIs
        for robot in documents['Robots']:
            if not documents['Robots'][robot]['type'] == 'virtual':
                self.get_logger().info('Crazyflie %s:: %s' % (documents['Robots'][robot]['name'], documents['Robots'][robot]['uri']))
                uris.add(documents['Robots'][robot]['uri'])

        # logging.basicConfig(level=logging.DEBUG)
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        self.cf_swarm = Swarm(uris, factory=factory)


        for robot in documents['Robots']:
            if not documents['Robots'][robot]['type'] == 'virtual':
                config = documents['Robots'][robot]
                id = config['name']
                self.get_logger().info('Crazyflie %s::%s' % (id, config['uri']))
                    
                cf = Crazyflie_ROS2(self, self.cf_swarm._cfs[config['uri']], config['uri'], id, config)
                dron.append(cf)
                # while not cf.scf.cf.param.is_updated:
                #     time.sleep(0.1)
                # self.get_logger().warning('Parameters downloaded for %s' % cf.scf.cf.link_uri)
        
        time.sleep(1.0)
        self.cf_swarm.parallel_safe(self.swarm_connection)
        time.sleep(1.0)
        self.cf_swarm.parallel_safe(self.update_params)

        for agent in dron:
            agent.load_formation_params()
            
    def update_params(self, scf):
        # Disable Flow deck to EKF
        if scf.CONTROL_MODE == 'None' or scf.CONTROL_MODE == 'Gimbal':
            scf.cf.param.set_value('motion.disable', '1')
        # Init Kalman Filter
        scf.cf.param.set_value('stabilizer.estimator', '2')
        # Set the std deviation for the quaternion data pushed into the
        # kalman filter. The default value seems to be a bit too low.
        scf.cf.param.set_value('locSrv.extQuatStdDev', 0.06)
        # Reset Estimator
        scf.cf.param.set_value('kalman.resetEstimation', '1')
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        # Init HighLevel
        scf.cf.param.set_value('commander.enHighLevel', '1')
        # Multi-Agent Robotic Control
        scf.cf.param.set_value('stabilizer.controller', '5')
    
    def swarm_connection(self, scf):
        self.get_logger().info('Connecting to %s' % scf.uri)
        scf.cf.open_link(scf.uri)
        while not scf.cf.param.is_updated:
            time.sleep(0.1)

    def order_callback(self, msg):
        self.get_logger().info('SWARM::Order: "%s"' % msg.data)
        if msg.data == 'take_off':
            for cf in dron:
                cf.take_off()
            self.swarm_ready = Timer(4, self._ready)
            self.swarm_ready.start()
        elif msg.data == 'land':
            for cf in dron:
                cf.descent()
        elif msg.data == 'formation_run':
            for cf in dron:
                cf.order_callback(msg)
            self.t_stop = Timer(20, self.stop_dataset)
            self.t_stop.start()
        elif msg.data == 'formation_stop':
            for cf in dron:
                cf.order_callback(msg)
        else:
            self.get_logger().error('SWARM::"%s": Unknown order' % msg.data)

    def stop_dataset(self):
        msg = String()
        msg.data = 'end'
        self.publisher_order.publish(msg)
        self.get_logger().info('Multi-Robot-System::Order: "%s"' % msg.data)

    def _ready(self):
        self.get_logger().info('SWARM::Ready!!')
        msg = String()
        msg.data = 'ready'
        self.publisher_status_.publish(msg)

    def goalpose_callback(self, msg):
        for cf in dron:
            cf.cmd_motion_.x = cf.cmd_motion_.x + msg.position.x
            cf.cmd_motion_.y = cf.cmd_motion_.y + msg.position.y
            cf.cmd_motion_.z = cf.cmd_motion_.z + msg.position.z
            cf.cmd_motion_.ckeck_pose()

            cf.scf.cf.high_level_commander.go_to_target_pose(cf.cmd_motion_.x, cf.cmd_motion_.y, cf.cmd_motion_.z)

        self.get_logger().info('SWARM::New Goal pose: X:%0.2f \tY:%0.2f \tZ:%0.2f' % (msg.position.x, msg.position.y, msg.position.z))

    def newpose_callback(self, msg):
        self.pose = msg


def main(args=None):
    rclpy.init(args=args)
    swarm_driver = CFSwarmDriver()
    rclpy.spin(swarm_driver)

    swarm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
