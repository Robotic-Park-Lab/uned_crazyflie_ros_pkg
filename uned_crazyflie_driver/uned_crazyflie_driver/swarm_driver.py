import logging
import time
import rclpy
from threading import Timer
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import os

from rclpy.node import Node
from std_msgs.msg import String, UInt16, UInt16MultiArray, Float64, Float64MultiArray
from geometry_msgs.msg import Pose, Twist, Point, TransformStamped
from visualization_msgs.msg import Marker
from uned_crazyflie_config.msg import Pidcontroller
from tf2_ros import TransformBroadcaster
from math import cos, sin, degrees, radians, pi, sqrt

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory, Swarm

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
        if d == None:
            self.x = x
            self.y = y
            self.z = z
            self.node.get_logger().info('Agent: %s' % self.str_())
        else:
            self.d = d
            self.distance = True
            self.node.get_logger().info('Agent: %d %s' % (self.idn, self.str_distance_()))
        self.pose = Pose()
        if self.id == 'origin':
            self.pose.position.x = 0.5
            self.pose.position.y = 0.0
            self.pose.position.z = 0.0
            self.k = 4.0
        else:
            self.sub_pose_ = self.node.create_subscription(Pose, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        
        parent.scf.cf.high_level_commander.new_neighbour(self.idn, self.d, self.k)
        self.publisher_data_ = self.node.create_publisher(Float64, self.parent.id + '/' + self.id + '/data', 10)
        self.publisher_iae_ = self.node.create_publisher(Float64, self.parent.id + '/' + self.id + '/iae', 10)
        self.publisher_marker_ = self.node.create_publisher(Marker, self.parent.id + '/' + self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))
    
    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def gtpose_callback(self, msg):
        self.pose = msg
        self.parent.scf.cf.high_level_commander.update_neighbour(self.idn, self.pose.position.x, self.pose.position.y, self.pose.position.z)
        self.node.get_logger().debug('Agent: X: %.2f Y: %.2f Z: %.2f' % (msg.position.x, msg.position.y, msg.position.z))

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
        self.logger.info('Command: %s' % self.str_())
        cf.commander.send_setpoint(self.roll, self.pitch, self.yaw, self.thrust)

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
    def __init__(self, parent, scf, link_uri, id, config, DT = False):
        self.scf = scf
        self.parent = parent
        self.config = config
        self.id = id
        self.parent.get_logger().info('Connecting to %s' % self.id)
        self.ready = False
        self.swarm_ready = False
        self.digital_twin = DT
        self.distance_formation_bool = False
        self.pose = Pose()
        self.home = Pose()
        
        self.init_pose = False
        self.tfbr = TransformBroadcaster(self.parent)
        self.parent.get_logger().info('Connecting to %s' % link_uri)
        self.scf.cf.connected.add_callback(self._connected)
        self.scf.cf.disconnected.add_callback(self._disconnected)
        self.scf.cf.connection_failed.add_callback(self._connection_failed)
        self.scf.cf.connection_lost.add_callback(self._connection_lost)
        self.scf.cf.open_link(link_uri)
        self.scf.CONTROL_MODE = self.config['control_mode']
        self.parent.get_logger().info('CF%s::Control Mode: %s!' % (self.scf.cf.link_uri[-2:], self.scf.CONTROL_MODE))
        self.scf.CONTROLLER_TYPE = self.config['controller_type']
        self.parent.get_logger().info('CF%s::Controller Type: %s!' % (self.scf.cf.link_uri[-2:], self.scf.CONTROLLER_TYPE))
        self.communication = (self.config['communication']['type'] == 'Continuous')
        if not self.communication:
            self.threshold = config['communication']['threshold']['co']
        else:
            self.threshold = 0.001

    def _connected(self, link_uri):
        self.parent.get_logger().info('Connected to %s -> Crazyflie %s' % (link_uri, self.scf.cf.link_uri[-2:]))
        # ROS
        # Publisher
        # POSE3D
        if self.config['local_pose']['enable']:
            if self.scf.CONTROL_MODE == 'None':
                self.publisher_pose = self.parent.create_publisher(Pose, self.id + '/pose', 10)
            else:
                self.publisher_pose = self.parent.create_publisher(Pose, self.id + '/local_pose', 10)
                if self.digital_twin:
                    self.publisher_dtpose = self.parent.create_publisher(Pose, self.id + '/pose_dt', 10)
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
        if self.config['data_rate']['enable']:
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

        self.publisher_global_iae_ = self.parent.create_publisher(Float64, self.id + '/global_iae', 10)
        # Subscription
        self.parent.create_subscription(String, self.id + '/order', self.order_callback, 10)
        self.parent.create_subscription(String, '/swarm/status', self.swarm_status_callback, 10)
        self.parent.create_subscription(Pose, self.id + '/target_pose', self.targetpose_callback, 10)
        if not self.scf.CONTROL_MODE == 'None':
            self.parent.create_subscription(Pose, self.id + '/pose', self.newpose_callback, 10)
        if self.scf.CONTROL_MODE == 'HighLevel':
            self.parent.create_subscription(Pose, self.id + '/goal_pose', self.goalpose_callback, 10)
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
        self.scf.cf.param.add_update_callback(group='controller', cb=self.param_stab_est_callback)
        # self.scf.cf.param.add_update_callback(group='commander', cb=self.param_stab_est_callback)
        # self.scf.cf.param.add_update_callback(group='stabilizer', cb=self.param_stab_est_callback)

        self._is_flying = False
        self.xy_lim = 1.5
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
        self.cmd_motion_.x = self.home.position.x
        self.cmd_motion_.y = self.home.position.y
        self.cmd_motion_.send_pose_data_(self.scf.cf)

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
            msg = Pose()
            msg.position.x = data['stateEstimate.x']
            msg.position.y = data['stateEstimate.y']
            msg.position.z = data['stateEstimate.z']
            roll = data['stabilizer.roll']
            pitch = data['stabilizer.pitch']
            yaw = data['stabilizer.yaw']
            msg.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            msg.orientation.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            msg.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            msg.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            delta = np.array([self.pose.position.x-msg.position.x,self.pose.position.y-msg.position.y,self.pose.position.z-msg.position.z])
            
            if self.communication or np.linalg.norm(delta)>self.threshold:
                self.pose = msg
                self.publisher_pose.publish(msg)
                if self.digital_twin:
                    self.publisher_dtpose.publish(msg)
                t_base = TransformStamped()
                t_base.header.stamp = self.parent.get_clock().now().to_msg()
                t_base.header.frame_id = 'map'
                t_base.child_frame_id = self.id+'/base_link'
                t_base.transform.translation.x = msg.position.x
                t_base.transform.translation.y = msg.position.y
                t_base.transform.translation.z = msg.position.z
                t_base.transform.rotation.x = msg.orientation.x
                t_base.transform.rotation.y = msg.orientation.y
                t_base.transform.rotation.z = msg.orientation.z
                t_base.transform.rotation.w = msg.orientation.w
                self.tfbr.sendTransform(t_base)
        else:
            try:
                if self.scf.cf.param.get_value('deck.bcLighthouse4') == '1' or self.config['positioning'] == 'Intern':
                    msg = Pose()
                    msg.position.x = data['stateEstimate.x']
                    msg.position.y = data['stateEstimate.y']
                    msg.position.z = data['stateEstimate.z']
                    self.init_pose = True
                    self.pose = msg
                    self.home = msg
                    self.publisher_pose.publish(msg)
                    self.cmd_motion_.x = msg.position.x
                    self.cmd_motion_.y = msg.position.y
                    self.cmd_motion_.z = msg.position.z
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
        elif msg.data == 'distance_formation_run':
            self.distance_formation_bool = True
            self.scf.cf.high_level_commander.enable_formation()
        elif msg.data == 'formation_stop':
            self.distance_formation_bool = False
            self.scf.cf.param.set_value('stabilizer.controller', '1')
            self.descent()
        else:
            self.parent.get_logger().error('CF%s::"%s": Unknown order' % (self.scf.cf.link_uri[-2:], msg.data))

    def cmd_control_callback(self, msg):
        if self.scf.CONTROL_MODE == 'OffBoard':
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
            self.pose = msg
            self.home = msg
            self.publisher_pose.publish(msg)
            self.scf.cf.extpos.send_extpos(msg.position.x, msg.position.y, msg.position.z)
            self.init_pose = True
            self.cmd_motion_.x = msg.position.x
            self.cmd_motion_.y = msg.position.y
            self.cmd_motion_.z = msg.position.z
            self.parent.get_logger().info('CF%s::Init pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
        x = np.array([self.pose.position.x-msg.position.x,self.pose.position.y-msg.position.y,self.pose.position.z-msg.position.z])
        if (np.linalg.norm(x)>0.005 and np.linalg.norm(x)<0.2):
            self.scf.cf.extpos.send_extpos(msg.position.x, msg.position.y, msg.position.z)
        if ((abs(msg.position.x)>self.xy_lim) or (abs(msg.position.y)>self.xy_lim) or (abs(msg.position.z)>2.0)) and self.scf.CONTROL_MODE != 'HighLevel':
            self.scf.CONTROL_MODE = 'HighLevel'
            self._is_flying = True
            self.parent.get_logger().error('CF%s::Out.' % self.scf.cf.link_uri[-2:])
            self.gohome()
            t_end = Timer(3, self.descent)
            t_end.start()

    def goalpose_callback(self, msg):
        if self.scf.CONTROL_MODE == 'HighLevel':
            self.cmd_motion_.x = msg.position.x
            self.cmd_motion_.y = msg.position.y
            self.cmd_motion_.z = msg.position.z
            # self.cmd_motion_.ckeck_pose()
            self.cmd_motion_.send_pose_data_(self.scf.cf)
            self.parent.get_logger().debug('CF%s::New Goal pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_())) 

    def targetpose_callback(self, msg):
        self.cmd_motion_.x = msg.position.x
        self.cmd_motion_.y = msg.position.y
        self.cmd_motion_.z = msg.position.z
        self.parent.get_logger().debug('CF%s::New Target pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
        self.scf.cf.high_level_commander.go_to_target_pose(msg.position.x, msg.position.y, msg.position.z)

    ###############
    #    Tasks    #
    ###############
    def load_formation_params(self):
        if self.config['task']['enable']:
            self.parent.get_logger().info('Task %s' % self.config['task']['type'])
            self.agent_list = list()
            aux = self.config['task']['relationship']
            self.relationship = aux.split(', ')
            if self.config['task']['type'] == 'distance':
                self.timer_task = self.parent.create_timer(0.1, self.task_formation_distance)
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, self.parent, aux[0], d = float(aux[1]))
                    self.agent_list.append(robot)
            if self.config['task']['type'] == 'relative_pose':
                self.timer_task = self.parent.create_timer(0.1, self.task_formation_pose)
                for rel in self.relationship:
                    aux = rel.split('_')
                    rel_pose = aux[1].split('/')
                    robot = Agent(self, self.parent, aux[0], x = float(rel_pose[0]), y = float(rel_pose[1]), z = float(rel_pose[2]))
                    self.agent_list.append(robot)

    def task_formation_distance(self):
        if self.distance_formation_bool:
            msg_iae = Float64()
            msg_iae.data = 0.0
            for agent in self.agent_list:
                if agent.id == 'origin':
                    distance = sqrt((pow(self.pose.position.x-0.5,2)+pow(self.pose.position.y,2)+pow(self.pose.position.z,2)))
                else:
                    distance = sqrt(pow(self.pose.position.x-agent.pose.position.x,2)+pow(self.pose.position.y-agent.pose.position.y,2)+pow(self.pose.position.z-agent.pose.position.z,2))
                msg_data = Float64()
                msg_data.data = abs(agent.d - distance)
                agent.publisher_data_.publish(msg_data)
                msg_data.data = agent.last_iae + (agent.last_error + msg_data.data) * 0.1 /2
                agent.last_error = distance
                agent.publisher_iae_.publish(msg_data)
                agent.last_iae = msg_data.data
                msg_iae.data += abs(agent.d - distance)

            self.publisher_global_iae_.publish(msg_iae)

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
        self.declare_parameter('enviroment', 'swarm')
        self.declare_parameter('robots', 'dron01')

        # Publisher
        self.publisher_status_ = self.create_publisher(String,'/swarm/status', 10)
        # Subscription
        self.sub_order = self.create_subscription(String, '/swarm/order', self.order_callback, 10)
        self.sub_pose_ = self.create_subscription(Pose, '/swarm/pose', self.newpose_callback, 10)
        self.sub_goal_pose_ = self.create_subscription(Pose, '/swarm/goal_pose', self.goalpose_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('SwarmDriver::inicialize() ok.')
        enviroment = self.get_parameter('enviroment').get_parameter_value().string_value
        if enviroment == 'swarm':
            config_package_dir = get_package_share_directory('uned_crazyflie_config')
        else:
            config_package_dir = get_package_share_directory('uned_swarm_config')
        self.pose = Pose()
        # Read Params
        aux = self.get_parameter('robots').get_parameter_value().string_value
        robot_list = aux.split(', ')
        n_robot = len(robot_list)
        config_file = self.get_parameter('config').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
            
        # Define crazyflie URIs
        for i in range(int(n_robot),0,-1):
            aux = documents[robot_list[i-1]]
            self.get_logger().info('Crazyflie %s:: %s' % (aux['name'], aux['uri']))
            uris.add(aux['uri'])

        # logging.basicConfig(level=logging.DEBUG)
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        self.cf_swarm = Swarm(uris, factory=factory)

        for i in range(int(n_robot),0,-1):
            aux = documents[robot_list[i-1]]
            id = aux['name']
            self.get_logger().info('Crazyflie %s:: %s' % (id, aux['uri']))
            config_path = os.path.join(config_package_dir, 'resources', aux['config_path'])
            with open(config_path, 'r') as file:
                aux_conf = yaml.safe_load(file)
            if aux['config_path'] == 'crazyflie_intern_default.yaml' or aux['config_path'] == 'crazyflie_extern_default.yaml' or aux['config_path'] == 'crazyflie_intern_event_default.yaml' or aux['config_path'] == 'crazyflie_extern_event_default.yaml':
                config =  aux_conf['dronXX']
            else:
                config =  aux_conf[id]
                
            cf = Crazyflie_ROS2(self, self.cf_swarm._cfs[aux['uri']], aux['uri'], id, config, DT = aux['type'] == 'digital_twin')
            dron.append(cf)
            while not cf.scf.cf.param.is_updated:
                time.sleep(0.1)
            self.get_logger().warning('Parameters downloaded for %s' % cf.scf.cf.link_uri)
        
        self.cf_swarm.parallel_safe(self.update_params)
        for agent in dron:
            agent.load_formation_params()
            

    def update_params(self, scf):
        
        # Disable Flow deck to EKF
        if scf.CONTROL_MODE == 'None':
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
        if (scf.CONTROLLER_TYPE == 'PID_EventBased'):
            scf.cf.param.set_value('stabilizer.controller', '5')
        scf.cf.param.set_value('stabilizer.controller', '5')
        
        

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
        elif msg.data == 'distance_formation_run':
            self._ready()
            for cf in dron:
                cf.order_callback(msg)
        elif msg.data == 'formation_stop':
            for cf in dron:
                cf.order_callback(msg)
        else:
            self.get_logger().error('SWARM::"%s": Unknown order' % msg.data)

    def _ready(self):
        self.get_logger().info('SWARM::Ready!!')
        msg = String()
        msg.data = 'Ready'
        self.publisher_status_.publish(msg)

    def goalpose_callback(self, msg):
        for cf in dron:
            cf.cmd_motion_.x = cf.cmd_motion_.x + msg.position.x
            cf.cmd_motion_.y = cf.cmd_motion_.y + msg.position.y
            cf.cmd_motion_.z = cf.cmd_motion_.z + msg.position.z
            # cf.cmd_motion_.ckeck_pose()
            delta = [abs(msg.position.x), abs(msg.position.y), abs(msg.position.z)]
            self.cmd_motion_.flight_time = max(delta)/self.max_vel
            # self.cmd_motion_.flight_time = 0.5
            cf.cmd_motion_.send_pose_data_(cf.scf.cf)

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
