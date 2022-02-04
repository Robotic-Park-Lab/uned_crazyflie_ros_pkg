import logging
import time
import rclpy
from threading import Timer
import numpy as np

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
from vicon_receiver.msg import Position

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

end_test = False
not_fail = True
xy_warn = 1.0
xy_lim = 1.3
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


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
        # self.logger.info('Goal Pose: %s' % self.pose_str_())
        cf.commander.send_position_setpoint(self.x, self.y, self.z, self.yaw)

    def send_offboard_setpoint_(self, cf):
        self.logger.info('Command: %s' % self.str_())
        cf.commander.send_setpoint(self.roll, self.pitch, self.yaw,
                                   self.thrust)


class Logging:
    def __init__(self, link_uri, parent):

        self._cf = Crazyflie(rw_cache='./cache')
        self.parent = parent
        self.parent.get_logger().info('Connecting to %s' % link_uri)
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        self.parent.get_logger().info('Connected to %s' % link_uri)
        # POSE3D
        self._lg_stab_pose = LogConfig(name='Pose', period_in_ms=10)
        self._lg_stab_pose.add_variable('stateEstimate.x', 'float')
        self._lg_stab_pose.add_variable('stateEstimate.y', 'float')
        self._lg_stab_pose.add_variable('stateEstimate.z', 'float')
        self._lg_stab_pose.add_variable('stabilizer.roll', 'float')
        self._lg_stab_pose.add_variable('stabilizer.pitch', 'float')
        self._lg_stab_pose.add_variable('stabilizer.yaw', 'float')
        # TWIST
        self._lg_stab_twist = LogConfig(name='Twist', period_in_ms=10)
        self._lg_stab_twist.add_variable('gyro.x', 'float')
        self._lg_stab_twist.add_variable('gyro.y', 'float')
        self._lg_stab_twist.add_variable('gyro.z', 'float')
        self._lg_stab_twist.add_variable('stateEstimate.vx', 'float')
        self._lg_stab_twist.add_variable('stateEstimate.vy', 'float')
        self._lg_stab_twist.add_variable('stateEstimate.vz', 'float')
        # Other data.
        self._lg_stab_data = LogConfig(name='Data', period_in_ms=10)
        self._lg_stab_data.add_variable('posEbCtl.Zcount', 'uint16_t')
        self._lg_stab_data.add_variable('posEbCtl.Ycount', 'uint16_t')
        self._lg_stab_data.add_variable('posEbCtl.Xcount', 'uint16_t')
        # self._lg_stab_data.add_variable('pm.vbat', 'FP16')
        # Params
        self._cf.param.add_update_callback(group='posCtlPid', cb=self.param_stab_est_callback)
        self._cf.param.add_update_callback(group='velCtlPid', cb=self.param_stab_est_callback)
        self._cf.param.add_update_callback(group='posEbCtlPid', cb=self.param_stab_est_callback)
        self._cf.param.add_update_callback(group='velEbCtlPid', cb=self.param_stab_est_callback)
        self._cf.param.add_update_callback(group='pid_attitude', cb=self.param_stab_est_callback)
        self._cf.param.add_update_callback(group='pid_rate', cb=self.param_stab_est_callback)
        self._cf.param.add_update_callback(group='controller', cb=self.param_stab_est_callback)

        # self._cf.param.add_update_callback(group='deck', cb=self.param_stab_est_callback)
        try:
            self._cf.log.add_config(self._lg_stab_pose)
            self._cf.log.add_config(self._lg_stab_twist)
            self._cf.log.add_config(self._lg_stab_data)
            # This callback will receive the data
            self._lg_stab_pose.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab_twist.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab_data.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab_pose.error_cb.add_callback(self._stab_log_error)
            self._lg_stab_twist.error_cb.add_callback(self._stab_log_error)
            self._lg_stab_data.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab_pose.start()
            self._lg_stab_twist.start()
            self._lg_stab_data.start()
        except KeyError as e:
            self.parent.get_logger().info('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            self.parent.get_logger().error('Could not add some log config, bad configuration.')

    def _stab_log_error(self, logconf, msg):
        self.parent.get_logger().error('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        if(logconf.name == "Pose"):
            self.parent.pose_callback(data)
        elif(logconf.name == "Twist"):
            self.parent.twist_callback(data)
        elif(logconf.name == "Data"):
            self.parent.data_callback(data)
            # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        else:
            self.parent.get_logger().error('Error: %s: not valid logconf' % logconf.name)

    def param_stab_est_callback(self, name, value):
        self.parent.get_logger().info('Parameter %s: %s' %(name, value))

    def _connection_failed(self, link_uri, msg):
        self.parent.get_logger().error('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        self.parent.get_logger().error('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        self.parent.get_logger().warning('Disconnected from %s' % link_uri)
        self.is_connected = False
        rclpy.shutdown()


class CFDriver(Node):
    def __init__(self):
        super().__init__('cf_driver')
        # Params
        self.declare_parameter('cf_uri', 'radio://0/80/2M/E7E7E7E701')
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
        # Publisher
        self.publisher_pose = self.create_publisher(Pose, 'cf_pose', 10)
        self.publisher_twist = self.create_publisher(Twist, 'cf_twist', 10)
        self.publisher_data = self.create_publisher(UInt16MultiArray, 'cf_data', 10)
        # Subscription
        self.sub_order = self.create_subscription(String, 'cf_order', self.order_callback, 10)
        self.sub_pose = self.create_subscription(Pose, 'pose', self.newpose_callback, 10)
        self.sub_goal_pose = self.create_subscription(Pose, 'goal_pose', self.goalpose_callback, 10)
        self.sub_cmd = self.create_subscription(Float64MultiArray, 'onboard_cmd', self.cmd_control_callback, 10)
        self.sub_controller = self.create_subscription(Pidcontroller, 'controllers_params', self.controllers_params_callback, 10)

        timer_period = 0.01  # seconds
        self.iterate_loop = self.create_timer(timer_period, self.iterate)
        self.initialize()

    def initialize(self):
        self.get_logger().info('CrazyflieDriver::inicialize() ok.')
        # Read & Check ROS Parameters
        dron_id = self.get_parameter('cf_uri').get_parameter_value().string_value
        self.get_logger().info('Crazyflie ID: %s!' % dron_id)
        self.CONTROL_MODE = self.get_parameter('cf_control_mode').get_parameter_value().string_value
        self.get_logger().info('Crazyflie Control Mode: %s!' % self.CONTROL_MODE)
        self.CONTROLLER_TYPE = self.get_parameter('cf_controller_type').get_parameter_value().string_value
        self.get_logger().info('Crazyflie Controller Type: %s!' % self.CONTROLLER_TYPE)
        # Connection
        cflib.crtp.init_drivers()
        available = cflib.crtp.scan_interfaces()
        for i in available:
            self.get_logger().info("Interface with URI [%s] found and name/comment [%s]" % (i[0], i[1]))
        self.scf = Logging(dron_id, self)
        # Init Variables
        self.scf.init_pose = False
        if self.CONTROL_MODE == 'OffBoard':
            self.scf._is_flying = True
        else:
            self.scf._is_flying = False
        self.cmd_motion_ = CMD_Motion(self.get_logger())
        self.scf._cf.commander.set_client_xmode(True)
        time.sleep(2.0)
        # Disable Flow deck to EKF
        # self.scf._cf.param.set_value('motion.disable', '1')
        # Init Kalman Filter
        self.scf._cf.param.set_value('stabilizer.estimator', '2')
        # Set the std deviation for the quaternion data pushed into the
        # kalman filter. The default value seems to be a bit too low.
        self.scf._cf.param.set_value('locSrv.extQuatStdDev', 0.06)
        # Reset Estimator
        self.scf._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.scf._cf.param.set_value('kalman.resetEstimation', '0')
        # Init Motors
        self.scf._cf.commander.send_setpoint(0.0, 0.0, 0, 0)
        # Controller type
        if (self.CONTROLLER_TYPE == 'EventBased'):
            self.scf._cf.param.set_value('controller.eventBased', '1')

    def take_off(self):
        self.get_logger().info('CrazyflieDriver::Take Off.')
        self.cmd_motion_.z = self.cmd_motion_.z + 0.7
        self.scf._is_flying = True

    def gohome(self):
        self.get_logger().info('CrazyflieDriver::Go Home.')
        self.cmd_motion_.x = 0.0
        self.cmd_motion_.y = 0.0

    def descent(self):
        self.cmd_motion_.z = 0.15
        self.get_logger().info('CrazyflieDriver::Descent.')
        t_desc = Timer(2, self.take_land)
        t_desc.start()

    def take_land(self):
        self.get_logger().info('CrazyflieDriver::Take Land.')
        self.cmd_motion_.z = 0.0
        self.scf._cf.commander.send_setpoint(0.0, 0.0, 0, 0)
        self.scf._cf.commander.send_stop_setpoint()
        self.scf.init_pose = False
        self.scf._is_flying = False

    def iterate(self):
        if self.CONTROL_MODE == 'HighLevel':
            if (self.cmd_motion_.z > 0.05 and self.scf._is_flying):
                self.cmd_motion_.send_pose_data_(self.scf._cf)
        elif self.CONTROL_MODE == 'OffBoard':
            self.cmd_motion_.send_offboard_setpoint_(self.scf._cf)

    def pose_callback(self, data):
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
        self.publisher_pose.publish(msg)

    def twist_callback(self, data):
        msg = Twist()
        msg.linear.x = data['stateEstimate.vx']
        msg.linear.y = data['stateEstimate.vy']
        msg.linear.z = data['stateEstimate.vz']
        msg.angular.x = data['gyro.x']
        msg.angular.y = data['gyro.y']
        msg.angular.z = data['gyro.z']
        self.publisher_twist.publish(msg)

    def data_callback(self, data):
        msg = UInt16MultiArray()
        msg.data = {data['posEbCtl.Xcount'], data['posEbCtl.Ycount'], data['posEbCtl.Zcount']}
        self.publisher_data.publish(msg)

    def order_callback(self, msg):
        self.get_logger().info('Order: "%s"' % msg.data)
        if msg.data == 'take_off':
            if self.scf._is_flying:
                self.get_logger().warning('Already flying')
            else:
                self.take_off()
        elif msg.data == 'land':
            if self.scf._is_flying:
                self.descent()
            else:
                self.get_logger().warning('In land')
        elif msg.data == 'gohome':
            if self.scf._is_flying:
                self.gohome()
            else:
                self.get_logger().warning('In land')
        else:
            self.get_logger().error('"%s": Unknown order' % msg.data)

    def cmd_control_callback(self, msg):
        if self.CONTROL_MODE == 'OffBoard':
            self.cmd_motion_.roll = msg.data[1]
            self.cmd_motion_.pitch = msg.data[2]
            self.cmd_motion_.yaw = msg.data[3]
            self.cmd_motion_.thrust = int(msg.data[0])
            self.get_logger().debug('Command: %s' % self.cmd_motion_.str_())
        else:
            self.get_logger().warning('New command control order. Offboard control disabled')

    def controllers_params_callback(self, msg):
        self.get_logger().info('New %s controller parameters' % msg.id)
        if (self.CONTROLLER_TYPE == 'Continuous'):
            if msg.id == 'x':
                groupstr = 'posCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'yVelMax', msg.upperlimit)
            elif msg.id == 'y':
                groupstr = 'posCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf._cf.param.set_value(groupstr + '.x' + msg.id + 'VelMax', msg.upperlimit)
            elif msg.id == 'z':
                groupstr = 'posCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'VelMax', msg.upperlimit)
            elif msg.id == 'vx':
                groupstr = 'velCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
            elif msg.id == 'vy':
                groupstr = 'velCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
            elif msg.id == 'vz':
                groupstr = 'velCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
            elif msg.id == 'roll':
                groupstr = 'pid_attitude'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'pitch':
                groupstr = 'pid_attitude'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'yaw':
                groupstr = 'pid_attitude'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'droll':
                groupstr = 'pid_rate'
                self.scf._cf.param.set_value(groupstr + '.' + 'roll_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + 'roll_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + 'roll_kd', msg.kd)
            elif msg.id == 'dpitch':
                groupstr = 'pid_rate'
                self.scf._cf.param.set_value(groupstr + '.' + 'pitch_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + 'pitch_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + 'pitch_kd', msg.kd)
            elif msg.id == 'dyaw':
                groupstr = 'pid_rate'
                self.scf._cf.param.set_value(groupstr + '.' + 'yaw_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + 'yaw_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + 'yaw_kd', msg.kd)
            self.get_logger().info('Kp: %0.2f \t Ki: %0.2f \t Kd: %0.2f \t N: %0.2f \t UL: %0.2f \t LL: %0.2f' % (msg.kp, msg.ki, msg.kd, msg.nd, msg.upperlimit, msg.lowerlimit))
        elif (self.CONTROLLER_TYPE == 'EventBased'):
            if msg.id == 'x':
                groupstr = 'posEbCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'yVelMax', msg.upperlimit)
            elif msg.id == 'y':
                groupstr = 'posEbCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
                self.scf._cf.param.set_value(groupstr + '.x' + msg.id + 'VelMax', msg.upperlimit)
            elif msg.id == 'z':
                groupstr = 'posEbCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'VelMax', msg.upperlimit)
            elif msg.id == 'vx':
                groupstr = 'velEbCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
            elif msg.id == 'vy':
                groupstr = 'velEbCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
            elif msg.id == 'vz':
                groupstr = 'velEbCtlPid'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Kd', msg.kd)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Co', msg.co)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + 'Ai', msg.ai)
            elif msg.id == 'roll':
                groupstr = 'pid_attitude'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'pitch':
                groupstr = 'pid_attitude'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'yaw':
                groupstr = 'pid_attitude'
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + msg.id + '_kd', msg.kd)
            elif msg.id == 'droll':
                groupstr = 'pid_rate'
                self.scf._cf.param.set_value(groupstr + '.' + 'roll_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + 'roll_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + 'roll_kd', msg.kd)
            elif msg.id == 'dpitch':
                groupstr = 'pid_rate'
                self.scf._cf.param.set_value(groupstr + '.' + 'pitch_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + 'pitch_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + 'pitch_kd', msg.kd)
            elif msg.id == 'dyaw':
                groupstr = 'pid_rate'
                self.scf._cf.param.set_value(groupstr + '.' + 'yaw_kp', msg.kp)
                self.scf._cf.param.set_value(groupstr + '.' + 'yaw_ki', msg.ki)
                self.scf._cf.param.set_value(groupstr + '.' + 'yaw_kd', msg.kd)
            self.get_logger().info('Kp: %0.2f \t Ki: %0.2f \t Kd: %0.2f \t N: %0.2f \t UL: %0.2f \t LL: %0.2f' % (msg.kp, msg.ki, msg.kd, msg.nd, msg.upperlimit, msg.lowerlimit))

    def newpose_callback(self, msg):
        self.scf._cf.extpos.send_extpos(msg.position.x, msg.position.y, msg.position.z)
        if not self.scf.init_pose:
            self.scf.init_pose = True
            self.cmd_motion_.x = msg.position.x
            self.cmd_motion_.y = msg.position.y
            self.cmd_motion_.z = msg.position.z
            self.get_logger().info('Init pose: %s' % self.cmd_motion_.pose_str_())
        if ((abs(msg.position.x)>xy_lim) or (abs(msg.position.y)>xy_lim) or (abs(msg.position.z)>2.0)) and self.CONTROL_MODE != 'HighLevel':
            self.CONTROL_MODE = 'HighLevel'
            self.scf._is_flying = True
            self.get_logger().error('CrazyflieDriver::Out.')
            self.gohome()
            t_end = Timer(3, self.descent)
            t_end.start()

    def goalpose_callback(self, msg):
        if self.CONTROL_MODE == 'HighLevel':
            self.cmd_motion_.x = msg.position.x
            self.cmd_motion_.y = msg.position.y
            self.cmd_motion_.z = msg.position.z
            self.cmd_motion_.ckeck_pose()
            self.get_logger().info('New Goal pose: %s' % self.cmd_motion_.pose_str_())
#


def main(args=None):
    rclpy.init(args=args)
    cf_driver = CFDriver()
    rclpy.spin(cf_driver)

    cf_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
