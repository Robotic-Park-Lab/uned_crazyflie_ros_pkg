import logging
import time
import rclpy
from threading import Timer
import numpy as np
import math

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from uned_crazyflie_config.msg import StateEstimate
from uned_crazyflie_config.msg import Pidcontroller
from uned_crazyflie_config.srv import AddTwoInts
from vicon_receiver.msg import Position

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# List of URIs, comment the one you do not want to fly
uris = set()
dron = list()
publisher = set()
xy_warn = 1.3
xy_lim = 1.5

class CMD_Motion():
    def __init__(self, logger):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0
        self.thrust = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.logger = logger
        self.flight_time = 1.0

    def ckeck_pose(self):
        # X Check
        if abs(self.x) > xy_warn:
            if abs(self.x) > xy_lim:
                self.logger.error('X: Error')
                if self.x > 0:
                    self.x = 0.95 * xy_warn
                else:
                    self.x = -0.95 * xy_warn
                self.logger.warning('New Point: %s' % self.pose_str_())
            else:
                self.logger.warning('X: Warning')
        # Y Check
        if abs(self.y) > xy_warn:
            if abs(self.y) > xy_lim:
                self.logger.error('Y: Error')
                if self.y > 0:
                    self.y = 0.95 * xy_warn
                else:
                    self.y = -0.95 * xy_warn
                self.logger.warning('New Point: %s' % self.pose_str_())
            else:
                self.logger.warning('Y: Warning')

    def str_(self):
        return ('Thrust: ' + str(self.thrust) + ' Roll: ' + str(self.roll) +
                ' Pitch: ' + str(self.pitch)+' Yaw: ' + str(self.yaw))

    def pose_str_(self):
        return ('X: ' + str(self.x) + ' Y: ' + str(self.y) +
                ' Z: ' + str(self.z)+' Yaw: ' + str(self.yaw))

    def send_pose_data_(self, cf):
        self.logger.info('Goal Pose: %s' % self.pose_str_())
        # cf.commander.send_position_setpoint(self.x, self.y, self.z, self.yaw)
        cf.high_level_commander.go_to(self.x, self.y, self.z, self.yaw, 0.5)

    def send_offboard_setpoint_(self, cf):
        self.logger.info('Command: %s' % self.str_())
        cf.commander.send_setpoint(self.roll, self.pitch, self.yaw,
                                   self.thrust)

############################
## CF Logging Class ##
############################
class CFLogging:
    def __init__(self, scf, parent, link_uri, ctrl_mode, ctrl_type, role):
        self.role = role
        self.parent = parent
        self.scf = scf
        self.id = 'dron' + link_uri[-2:]
        self.ready = False
        self.last_pose = Pose()
        self.parent.get_logger().info('Connecting to %s' % link_uri)
        self.parent.get_logger().info('%s role: %s!' % (self.id, self.role))
        self.scf.cf.connected.add_callback(self._connected)
        self.scf.cf.disconnected.add_callback(self._disconnected)
        self.scf.cf.connection_failed.add_callback(self._connection_failed)
        self.scf.cf.connection_lost.add_callback(self._connection_lost)
        self.scf.cf.open_link(link_uri)
        self.CONTROL_MODE = ctrl_mode
        self.parent.get_logger().info('CF%s::Control Mode: %s!' % (self.scf.cf.link_uri[-2:], self.CONTROL_MODE))
        self.scf.CONTROLLER_TYPE = ctrl_type
        self.parent.get_logger().info('CF%s::Controller Type: %s!' % (self.scf.cf.link_uri[-2:], self.scf.CONTROLLER_TYPE))

    def _connected(self, link_uri):
        self.parent.get_logger().info('Connected to %s -> Crazyflie %s' % (link_uri, self.scf.cf.link_uri[-2:]))
        # ROS
        # Publisher
        self.publisher_pose = self.parent.create_publisher(Pose, self.id + '/cf_pose', 10)
        self.publisher_twist = self.parent.create_publisher(Twist, self.id + '/cf_twist', 10)
        self.publisher_data = self.parent.create_publisher(UInt16MultiArray, self.id + '/cf_data', 10)
        self.publisher_data_attitude = self.parent.create_publisher(Float64MultiArray, self.id + '/cf_data_attitude', 10)
        self.publisher_data_rate = self.parent.create_publisher(Float64MultiArray, self.id + '/cf_data_rate', 10)
        self.publisher_data_motor = self.parent.create_publisher(Float64MultiArray, self.id + '/cf_data_motor', 10)
        # Subscription
        self.sub_order = self.parent.create_subscription(String, self.id + '/cf_order', self.order_callback, 10)
        self.sub_pose = self.parent.create_subscription(Pose, self.id + '/pose', self.newpose_callback, 10)
        self.sub_goal_pose = self.parent.create_subscription(Pose, self.id + '/goal_pose', self.goalpose_callback, 10)
        self.sub_cmd = self.parent.create_subscription(Float64MultiArray, self.id + '/onboard_cmd', self.cmd_control_callback, 10)
        self.sub_controller = self.parent.create_subscription(Pidcontroller, self.id + '/controllers_params', self.controllers_params_callback, 10)
        # POSE3D
        self._lg_stab_pose = LogConfig(name='Pose', period_in_ms=10)
        self._lg_stab_pose.add_variable('stateEstimate.x', 'float')
        self._lg_stab_pose.add_variable('stateEstimate.y', 'float')
        self._lg_stab_pose.add_variable('stateEstimate.z', 'float')
        self._lg_stab_pose.add_variable('stabilizer.roll', 'float')
        self._lg_stab_pose.add_variable('stabilizer.pitch', 'float')
        self._lg_stab_pose.add_variable('stabilizer.yaw', 'float')
        # TWIST
        '''
        self._lg_stab_twist = LogConfig(name='Twist', period_in_ms=10)
        self._lg_stab_twist.add_variable('gyro.x', 'float')
        self._lg_stab_twist.add_variable('gyro.y', 'float')
        self._lg_stab_twist.add_variable('gyro.z', 'float')
        self._lg_stab_twist.add_variable('stateEstimate.vx', 'float')
        self._lg_stab_twist.add_variable('stateEstimate.vy', 'float')
        self._lg_stab_twist.add_variable('stateEstimate.vz', 'float')
        '''
        # DATA ATTITUDE.
        self._lg_stab_data_a = LogConfig(name='Data_attitude', period_in_ms=10)
        self._lg_stab_data_a.add_variable('posCtl.targetVX', 'float')
        self._lg_stab_data_a.add_variable('posCtl.targetVY', 'float')
        self._lg_stab_data_a.add_variable('controller.roll', 'float')
        self._lg_stab_data_a.add_variable('controller.pitch', 'float')
        self._lg_stab_data_a.add_variable('controller.yaw', 'float')
        # DATA RATE.
        self._lg_stab_data_r = LogConfig(name='Data_rate', period_in_ms=10)
        self._lg_stab_data_r.add_variable('controller.rollRate', 'float')
        self._lg_stab_data_r.add_variable('controller.pitchRate', 'float')
        self._lg_stab_data_r.add_variable('controller.yawRate', 'float')
        self._lg_stab_data_r.add_variable('controller.cmd_roll', 'float')
        self._lg_stab_data_r.add_variable('controller.cmd_pitch', 'float')
        self._lg_stab_data_r.add_variable('controller.cmd_yaw', 'float')
        # DATA MOTOR.
        self._lg_stab_data_m = LogConfig(name='Data_motor', period_in_ms=10)
        self._lg_stab_data_m.add_variable('posCtl.targetVZ', 'float')
        self._lg_stab_data_m.add_variable('controller.cmd_thrust', 'float')
        self._lg_stab_data_m.add_variable('motor.m1', 'float')
        self._lg_stab_data_m.add_variable('motor.m2', 'float')
        self._lg_stab_data_m.add_variable('motor.m3', 'float')
        self._lg_stab_data_m.add_variable('motor.m4', 'float')
        # Other data.
        '''
        self._lg_stab_data = LogConfig(name='Data', period_in_ms=50)
        self._lg_stab_data.add_variable('posEbCtl.Zcount', 'uint16_t')
        self._lg_stab_data.add_variable('posEbCtl.Ycount', 'uint16_t')
        self._lg_stab_data.add_variable('posEbCtl.Xcount', 'uint16_t')
        self._lg_stab_data.add_variable('pm.vbat', 'FP16')
        '''
        # Params
        '''
        self.scf.cf.param.add_update_callback(group='posCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='velCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='posEbCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='velEbCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='pid_attitude', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='pid_rate', cb=self.param_stab_est_callback)
        '''
        self.scf.cf.param.add_update_callback(group='deck', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='controller', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='commander', cb=self.param_stab_est_callback)

        try:
            self.scf.cf.log.add_config(self._lg_stab_pose)
            self._lg_stab_pose.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab_pose.error_cb.add_callback(self._stab_log_error)
            # self.scf.cf.log.add_config(self._lg_stab_twist)
            # self._lg_stab_twist.data_received_cb.add_callback(self._stab_log_data)
            # self._lg_stab_twist.error_cb.add_callback(self._stab_log_error)
            self.scf.cf.log.add_config(self._lg_stab_data_a)
            self._lg_stab_data_a.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab_data_a.error_cb.add_callback(self._stab_log_error)
            # self.scf.cf.log.add_config(self._lg_stab_data)
            # self._lg_stab_data.data_received_cb.add_callback(self._stab_log_data)
            # self._lg_stab_data.error_cb.add_callback(self._stab_log_error)
            self.scf.cf.log.add_config(self._lg_stab_data_r)
            self._lg_stab_data_r.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab_data_r.error_cb.add_callback(self._stab_log_error)
            self.scf.cf.log.add_config(self._lg_stab_data_m)
            self._lg_stab_data_m.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab_data_m.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab_pose.start()
            # self._lg_stab_twist.start()
            self._lg_stab_data_a.start()
            self._lg_stab_data_r.start()
            self._lg_stab_data_m.start()
            # self._lg_stab_data.start()
        except KeyError as e:
            self.parent.get_logger().info('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        self._is_flying = False
        self.init_pose = False
        self.cmd_motion_ = CMD_Motion(self.parent.get_logger())
        self.scf.cf.commander.set_client_xmode(True)

    def _stab_log_error(self, logconf, msg):
        self.parent.get_logger().error('Crazyflie %s. Error when logging %s: %s' % (self.scf.cf.link_uri[-2:], logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        if(logconf.name == "Pose"):
            self.pose_callback(data)
        elif(logconf.name == "Twist"):
            self.twist_callback(data)
        elif(logconf.name == "Data_attitude"):
            self.dataAttitude_callback(data)
        elif(logconf.name == "Data_rate"):
            self.dataRate_callback(data)
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

    def take_off(self):
        self.parent.get_logger().info('CF%s::Take Off.' % self.scf.cf.link_uri[-2:])
        self.cmd_motion_.z = 0.75
        self.cmd_motion_.send_pose_data_(self.scf.cf)
        self._is_flying = True
        self.t_ready = Timer(2, self._ready)
        self.t_ready.start()

    def _ready(self):
        self.parent.get_logger().info('CF%s::Ready!!.' % self.scf.cf.link_uri[-2:])
        self.ready = True

    def gohome(self):
        self.parent.get_logger().info('CF%s::Go Home.' % self.scf.cf.link_uri[-2:])
        self.cmd_motion_.x = 0.0
        self.cmd_motion_.y = 0.0
        self.cmd_motion_.send_pose_data_(self.scf.cf)

    def descent(self):
        self.cmd_motion_.z = 0.1
        self.cmd_motion_.send_pose_data_(self.scf.cf)
        self.parent.get_logger().info('CF%s::Descent.' % self.scf.cf.link_uri[-2:])
        self.t_desc = Timer(2, self.take_land)
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

            self.last_pose = msg
            self.publisher_pose.publish(msg)
            if(self.role == 'leader'):
                x = np.array([self.last_pose.position.x-msg.position.x,self.last_pose.position.y-msg.position.y,self.last_pose.position.z-msg.position.z])
                if (np.linalg.norm(x)>0.10):
                    self.last_pose = msg
                    self.parent.task_manager(self.id, msg.position.x, msg.position.y, msg.position.z, yaw, self.role)

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
        else:
            self.parent.get_logger().error('CF%s::"%s": Unknown order' % (self.scf.cf.link_uri[-2:], msg.data))

    def cmd_control_callback(self, msg):
        if self.CONTROL_MODE == 'OffBoard':
            self.cmd_motion_.roll = msg.data[1]
            self.cmd_motion_.pitch = msg.data[2]
            self.cmd_motion_.yaw = msg.data[3]
            self.cmd_motion_.thrust = int(msg.data[0])
            self.parent.get_logger().debug('CF%s::Command: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.str_()))
        else:
            self.parent.get_logger().warning('CF%s::New command control order. Offboard control disabled' % self.scf.cf.link_uri[-2:])

    def controllers_params_callback(self, msg):
        self.parent.get_logger().info('CF%s: New %s controller parameters' % (self.scf.cf.link_uri[-2:], msg.id))
        if (self.scf.CONTROLLER_TYPE == 'Continuous'):
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
        elif (self.scf.CONTROLLER_TYPE == 'EventBased'):
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

    def newpose_callback(self, msg):
        if not self.init_pose:
            self.last_pose = msg
            self.publisher_pose.publish(msg)
            self.scf.cf.extpos.send_extpos(msg.position.x, msg.position.y, msg.position.z)
            self.init_pose = True
            self.cmd_motion_.x = msg.position.x
            self.cmd_motion_.y = msg.position.y
            self.cmd_motion_.z = msg.position.z
            self.parent.get_logger().info('CF%s::Init pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
        x = np.array([self.last_pose.position.x-msg.position.x,self.last_pose.position.y-msg.position.y,self.last_pose.position.z-msg.position.z])
        if (np.linalg.norm(x)>0.025 and np.linalg.norm(x)>0.05):
            self.scf.cf.extpos.send_extpos(msg.position.x, msg.position.y, msg.position.z)
        if ((abs(msg.position.x)>xy_lim) or (abs(msg.position.y)>xy_lim) or (abs(msg.position.z)>2.0)) and self.CONTROL_MODE != 'HighLevel':
            self.CONTROL_MODE = 'HighLevel'
            self._is_flying = True
            self.parent.get_logger().error('CF%s::Out.' % self.scf.cf.link_uri[-2:])
            self.gohome()
            t_end = Timer(3, self.descent)
            t_end.start()

    def goalpose_callback(self, msg):
        if self.CONTROL_MODE == 'HighLevel':
            self.cmd_motion_.x = msg.position.x
            self.cmd_motion_.y = msg.position.y
            self.cmd_motion_.z = msg.position.z
            self.cmd_motion_.ckeck_pose()
            self.cmd_motion_.send_pose_data_(self.scf.cf)
            self.parent.get_logger().info('CF%s::New Goal pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))

#####################
## CF Swarm Class  ##
#####################
class CFSwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('cf_first_uri', 'radio://0/80/2M/E7E7E7E701')
        self.declare_parameter('cf_role', 'leader, follower')
        self.declare_parameter('cf_relationship', 'dron01-dron02')
        self.declare_parameter('cf_num_uri', 1)
        self.declare_parameter('cf_control_mode', 'HighLevel')
        """
        Test:
        HighLevel: HighLevel
        OffBoard: Trajectory + Position
        LowLevel: Trajectory + Position + Attitude
        """
        self.declare_parameter('cf_controller_type', 'EventBased')
        """
        Test: -
        Continuous: Continuous PID
        EventBased: Event Based PID
        """
        # Subscription
        self.sub_order = self.create_subscription(String, 'swarm/cf_order', self.order_callback, 10)
        # self.sub_pose = self.create_subscription(Pose, 'swarm/pose', self.newpose_callback, 10)
        self.sub_goal_pose = self.create_subscription(Pose, 'swarm/goal_pose', self.goalpose_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('Follower-Leader::inicialize() ok.')
        # Read Params
        dron_id = self.get_parameter('cf_first_uri').get_parameter_value().string_value
        n = self.get_parameter('cf_num_uri').get_parameter_value().integer_value
        aux = self.get_parameter('cf_control_mode').get_parameter_value().string_value
        control_mode = aux.split(', ')
        aux = self.get_parameter('cf_controller_type').get_parameter_value().string_value
        controller_type = aux.split(', ')
        aux = self.get_parameter('cf_role').get_parameter_value().string_value
        roles = aux.split(', ')
        aux = self.get_parameter('cf_relationship').get_parameter_value().string_value
        self.relationship = aux.split(', ')

        # Define crazyflie URIs
        id_address = dron_id[-10:]
        id_base = dron_id[:16]
        id_address_int = int(id_address, 16)
        for i in range(int(n),0,-1):
            cf_str = id_base + hex(id_address_int+i-1)[-10:].upper()
            uris.add(cf_str)
            print(uris)
            self.get_logger().info('Crazyflie %d URI: %s!' % (i, cf_str))

        print(uris)
        # logging.basicConfig(level=logging.DEBUG)
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        self.cf_swarm = Swarm(uris, factory=factory)
        i = 0
        for uri in uris:
            if(uri == 'radio://0/80/2M/E7E7E7E701'):
                roles[i] = 'leader'
            if(uri == 'radio://0/80/2M/E7E7E7E702'):
                roles[i] = 'leader'
            if(uri == 'radio://0/80/2M/E7E7E7E703'):
                roles[i] = 'follower'
            cf = CFLogging(self.cf_swarm._cfs[uri], self, uri, control_mode[i], controller_type[i], roles[i])
            dron.append(cf)
            print(uri)
            while not cf.scf.cf.param.is_updated:
                time.sleep(1.0)
            print('Parameters downloaded for', cf.scf.cf.link_uri)
            i += 1

        print('All Connected')
        self.cf_swarm.parallel_safe(self.update_params)

    def update_params(self, scf):
        # Disable Flow deck to EKF
        # scf.cf.param.set_value('motion.disable', '1')
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
        if (scf.CONTROLLER_TYPE == 'EventBased'):
            scf.cf.param.set_value('controller.eventBased', '1')

    def order_callback(self, msg):
        self.get_logger().info('SWARM::Order: "%s"' % msg.data)
        if msg.data == 'take_off':
            for cf in dron:
                cf.take_off()
        elif msg.data == 'land':
            for cf in dron:
                cf.descent()
        else:
            self.get_logger().error('SWARM::"%s": Unknown order' % msg.data)

    def goalpose_callback(self, msg):
        for cf in dron:
            cf.cmd_motion_.x = cf.cmd_motion_.x + msg.position.x
            cf.cmd_motion_.y = cf.cmd_motion_.y + msg.position.y
            cf.cmd_motion_.z = cf.cmd_motion_.z + msg.position.z
            cf.cmd_motion_.ckeck_pose()
            delta = [abs(msg.position.x), abs(msg.position.y), abs(msg.position.z)]
            self.cmd_motion_.flight_time = max(delta)/self.max_vel
            cf.cmd_motion_.send_pose_data_(cf.scf.cf)

        self.get_logger().info('SWARM::New Goal pose: X:%0.2f \tY:%0.2f \tZ:%0.2f' % (msg.position.x, msg.position.y, msg.position.z))

    def task_manager(self, id, x, y, z, yaw, role):
        self.get_logger().info('%s role: %s!' % (id, role))
        for rel in self.relationship:
            if(rel.find(id) == 0):
                for cf in dron:
                    if(cf.scf.cf.link_uri[-2:]==rel[-2:] and cf.ready):
                        self.get_logger().info('New goal pose to follower %s!' % rel[-6:])
                        msg = Pose()
                        msg.position.x = x + 0.25
                        msg.position.y = y + 0.25
                        msg.position.z = z + 0.1
                        cf.goalpose_callback(msg)
                        break


def main(args=None):
    rclpy.init(args=args)
    swarm_driver = CFSwarmDriver()
    rclpy.spin(swarm_driver)

    swarm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
