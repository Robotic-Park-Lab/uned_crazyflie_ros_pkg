from threading import Timer
import numpy as np
import rclpy
from math import sqrt

from std_msgs.msg import String, UInt16MultiArray, Float64MultiArray, Float64
from geometry_msgs.msg import Pose, TransformStamped, Twist
from uned_crazyflie_config.msg import Pidcontroller
from tf2_ros import TransformBroadcaster

from rclpy.node import Node
from cflib.crazyflie.log import LogConfig
from uned_crazyflie_driver.agent_class import Agent


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
        self.logger.info('Goal Pose: X: %.4f Y: %.4f Z: %.4f' % (self.x, self.y, self.z))
        # cf.commander.send_position_setpoint(self.x, self.y, self.z, self.yaw)
        cf.high_level_commander.go_to(self.x, self.y, self.z, self.yaw, 0.5)

    def send_offboard_setpoint_(self, cf):
        self.logger.info('Command: %s' % self.str_())
        cf.commander.send_setpoint(self.roll, self.pitch, self.yaw, self.thrust)

######################
## CF Logging Class ##
######################
class Crazyflie_ROS2():
    def __init__(self, scf, link_uri, ctrl_mode, ctrl_type, config):
        self.scf = scf
        self.id = 'dron' + link_uri[-2:]
        self.config = config
        # self.node = rclpy.create_node(self.id+'_driver', namespace='/my_ns', use_global_arguments=False)
        self.node = rclpy.create_node(self.id+'_driver', use_global_arguments=False)
        self.node.name = self.id
        self.ready = False
        self.swarm_ready = False
        self.distance_formation_bool = False
        self.pose = Pose()
        self.home = Pose()
        if self.config['task']['enable']:
            self.node.get_logger().info('Task %s' % self.config['task']['type'])
            self.agent_list = list()
            aux = self.config['task']['relationship']
            self.relationship = aux.split(', ')
            if self.config['task']['type'] == 'distance':
                self.timer_task = self.node.create_timer(1, self.task_formation_distance)
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, aux[0], d = float(aux[1]))
                    self.agent_list.append(robot)
        self.init_pose = False
        self.tfbr = TransformBroadcaster(self.node)
        self.node.get_logger().info('Connecting to %s' % link_uri)
        self.scf.cf.connected.add_callback(self._connected)
        self.scf.cf.disconnected.add_callback(self._disconnected)
        self.scf.cf.connection_failed.add_callback(self._connection_failed)
        self.scf.cf.connection_lost.add_callback(self._connection_lost)
        self.scf.cf.open_link(link_uri)
        self.CONTROL_MODE = ctrl_mode
        self.node.get_logger().info('CF%s::Control Mode: %s!' % (self.scf.cf.link_uri[-2:], self.CONTROL_MODE))
        self.scf.CONTROLLER_TYPE = ctrl_type
        self.node.get_logger().info('CF%s::Controller Type: %s!' % (self.scf.cf.link_uri[-2:], self.scf.CONTROLLER_TYPE))

    def _connected(self, link_uri):
        self.node.get_logger().info('Connected to %s -> Crazyflie %s' % (link_uri, self.scf.cf.link_uri[-2:]))
        # ROS
        # Publisher
        # POSE3D
        if self.config['local_pose']['enable']:
            self.publisher_pose = self.node.create_publisher(Pose, self.id + '/local_pose', 10)
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
                self.node.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.node.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # TWIST
        if self.config['local_twist']['enable']:
            self.publisher_twist = self.node.create_publisher(Twist, self.id + '/local_twist', 10)
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
                self.node.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.node.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # DATA ATTITUDE.
        if self.config['data_attitude']['enable']:
            self.publisher_data_attitude = self.node.create_publisher(Float64MultiArray, self.id + '/data_attitude', 10)
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
                self.node.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.node.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # DATA RATE.
        if self.config['data_rate']['enable']:
            self.publisher_data_rate = self.node.create_publisher(Float64MultiArray, self.id + '/data_rate', 10)
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
                self.node.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.node.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # DATA MOTOR.
        if self.config['data_rate']['enable']:
            self.publisher_data_motor = self.node.create_publisher(Float64MultiArray, self.id + '/data_motor', 10)
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
                self.node.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.node.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # DATA.
        if self.config['data']['enable']:
            self.publisher_data = self.node.create_publisher(UInt16MultiArray, self.id + '/data', 10)
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
                self.node.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.node.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # Subscription
        self.sub_order = self.node.create_subscription(String, self.id + '/order', self.order_callback, 10)
        self.sub_swarm_status = self.node.create_subscription(String, '/swarm/status', self.swarm_status_callback, 10)
        self.sub_pose = self.node.create_subscription(Pose, self.id + '/pose', self.newpose_callback, 10)
        if self.CONTROL_MODE == 'HighLevel':
            self.sub_goal_pose = self.node.create_subscription(Pose, self.id + '/goal_pose', self.goalpose_callback, 10)
        else:
            self.sub_cmd = self.node.create_subscription(Float64MultiArray, self.id + '/onboard_cmd', self.cmd_control_callback, 10)
        self.sub_controller = self.node.create_subscription(Pidcontroller, self.id + '/controllers_params', self.controllers_params_callback, 10)
        
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

        self._is_flying = False
        self.xy_lim = 1.5
        self.cmd_motion_ = CMD_Motion(self.node.get_logger(), xy_lim = self.xy_lim)
        self.scf.cf.commander.set_client_xmode(True)

    def _stab_log_error(self, logconf, msg):
        self.node.get_logger().error('Crazyflie %s. Error when logging %s: %s' % (self.scf.cf.link_uri[-2:], logconf.name, msg))

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
            self.node.get_logger().error('CF%s. Error: %s: not valid logconf' % (self.scf.cf.link_uri[-2:], logconf.name))

    def param_stab_est_callback(self, name, value):
        self.node.get_logger().info('CF%s. Parameter %s: %s' %(self.scf.cf.link_uri[-2:], name, value))

    def _connection_failed(self, link_uri, msg):
        self.node.get_logger().error('Crazyflie %s. Connection to %s failed: %s' % (self.scf.cf.link_uri[-2:], link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        self.node.get_logger().error('Crazyflie %s. Connection to %s lost: %s' % (self.scf.cf.link_uri[-2:], link_uri, msg))

    def _disconnected(self, link_uri):
        self.node.get_logger().warning('Crazyflie %s. Disconnected from %s' % (self.scf.cf.link_uri[-2:], link_uri))
        self.is_connected = False
        self.node.destroy_node()

    def take_off(self):
        self.node.get_logger().info('CF%s::Take Off.' % self.scf.cf.link_uri[-2:])
        self.cmd_motion_.z = 0.75
        self.cmd_motion_.send_pose_data_(self.scf.cf)
        self._is_flying = True
        self.t_ready = Timer(2, self._ready)
        self.t_ready.start()

    def _ready(self):
        self.node.get_logger().info('CF%s::Ready!!.' % self.scf.cf.link_uri[-2:])
        self.ready = True

    def gohome(self):
        self.node.get_logger().info('CF%s::Go Home.' % self.scf.cf.link_uri[-2:])
        self.cmd_motion_.x = self.home.position.x
        self.cmd_motion_.y = self.home.position.y
        self.cmd_motion_.send_pose_data_(self.scf.cf)

    def descent(self):
        self.cmd_motion_.z = 0.1
        self.cmd_motion_.send_pose_data_(self.scf.cf)
        self.node.get_logger().info('CF%s::Descent.' % self.scf.cf.link_uri[-2:])
        self.t_desc = Timer(2, self.take_land)
        self._is_flying = False
        self.ready = False
        self.t_desc.start()

    def take_land(self):
        self.node.get_logger().info('CF%s::Take Land.' % self.scf.cf.link_uri[-2:])
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

            self.pose = msg
            self.publisher_pose.publish(msg)

            t_base = TransformStamped()
            t_base.header.stamp = self.node.get_clock().now().to_msg()
            t_base.header.frame_id = 'map'
            t_base.child_frame_id = self.id
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
                if self.scf.cf.param.get_value('deck.bcLighthouse4') == '1':
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
                    self.node.get_logger().info('CF%s::Home pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
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

    def dataMotor_callback(self, data):
        msg = Float64MultiArray()
        msg.data = {data['posCtl.targetVZ'], data['controller.cmd_thrust'], data['motor.m1'], data['motor.m2'], data['motor.m3'], data['motor.m4']}
        self.publisher_data_motor.publish(msg)

    def data_callback(self, data):
        msg = UInt16MultiArray()
        msg.data = {data['posEbCtl.Xcount'], data['posEbCtl.Ycount'], data['posEbCtl.Zcount']}
        self.publisher_data.publish(msg)

    def order_callback(self, msg):
        self.node.get_logger().info('CF%s::Order: "%s"' % (self.scf.cf.link_uri[-2:], msg.data))
        if msg.data == 'take_off':
            if self._is_flying:
                self.node.get_logger().warning('CF%s::Already flying' % self.scf.cf.link_uri[-2:])
            else:
                self.take_off()
        elif msg.data == 'land':
            if self._is_flying:
                self.descent()
            else:
                self.node.get_logger().warning('CF%s::In land' % self.scf.cf.link_uri[-2:])
        elif msg.data == 'gohome':
            if self._is_flying:
                self.gohome()
            else:
                self.node.get_logger().warning('CF%s::In land' % self.scf.cf.link_uri[-2:])
        else:
            self.node.get_logger().error('CF%s::"%s": Unknown order' % (self.scf.cf.link_uri[-2:], msg.data))

    def cmd_control_callback(self, msg):
        if self.CONTROL_MODE == 'OffBoard':
            self.cmd_motion_.roll = msg.data[1]
            self.cmd_motion_.pitch = msg.data[2]
            self.cmd_motion_.yaw = msg.data[3]
            self.cmd_motion_.thrust = int(msg.data[0])
            self.node.get_logger().debug('CF%s::Command: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.str_()))
        else:
            self.node.get_logger().warning('CF%s::New command control order. Offboard control disabled' % self.scf.cf.link_uri[-2:])

    def controllers_params_callback(self, msg):
        self.node.get_logger().info('CF%s: New %s controller parameters' % (self.scf.cf.link_uri[-2:], msg.id))
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

    def swarm_status_callback(self, msg):
        self.swarm_ready = True

    def newpose_callback(self, msg):
        if not self.init_pose:
            self.pose = msg
            self.home = msg
            self.publisher_pose.publish(msg)
            self.scf.cf.extpos.send_extpos(msg.position.x, msg.position.y, msg.position.z)
            self.init_pose = True
            self.cmd_motion_.x = msg.position.x
            self.cmd_motion_.y = msg.position.y
            self.cmd_motion_.z = msg.position.z
            self.node.get_logger().info('CF%s::Init pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
        x = np.array([self.pose.position.x-msg.position.x,self.pose.position.y-msg.position.y,self.pose.position.z-msg.position.z])
        if (np.linalg.norm(x)>0.005 and np.linalg.norm(x)>0.05):
            self.scf.cf.extpos.send_extpos(msg.position.x, msg.position.y, msg.position.z)
        if ((abs(msg.position.x)>self.xy_lim) or (abs(msg.position.y)>self.xy_lim) or (abs(msg.position.z)>2.0)) and self.CONTROL_MODE != 'HighLevel':
            self.CONTROL_MODE = 'HighLevel'
            self._is_flying = True
            self.node.get_logger().error('CF%s::Out.' % self.scf.cf.link_uri[-2:])
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
            # self.node.get_logger().info('CF%s::New Goal pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))

    ###############
    #    Tasks    #
    ###############
    def task_formation_distance(self):
        if self.ready and self.swarm_ready and self.distance_formation_bool:
            self.distance_formation_bool = False
            dx = dy = dz = 0
            target_pose = Pose()
            for agent in self.agent_list:
                # self.node.get_logger().info('Agent: %s' % agent.id)
                error_x = self.pose.position.x - agent.pose.position.x
                error_y = self.pose.position.y - agent.pose.position.y
                error_z = self.pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                dx += (pow(agent.d,2) - distance) * error_x
                dy += (pow(agent.d,2) - distance) * error_y
                dz += (pow(agent.d,2) - distance) * error_z

                msg_data = Float64()
                msg_data.data = agent.d - sqrt(distance)
                agent.publisher_data_.publish(msg_data)

            delta = sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))

            target_pose.position.x = self.pose.position.x + dx/4
            target_pose.position.y = self.pose.position.y + dy/4
            target_pose.position.z = self.pose.position.z # + dz/4
            
            self.goalpose_callback(target_pose)

            self.node.get_logger().debug('Distance: %.4f eX: %.2f eY: %.2f eZ: %.2f' % (sqrt(distance), error_x, error_y, error_z))
            self.node.get_logger().debug('Delta: %.4f X: %.2f Y: %.2f Z: %.2f' % (delta, dx, dy, dz))
            self.node.get_logger().debug('Target: X: %.2f Y: %.2f Z: %.2f' % (target_pose.position.x, target_pose.position.y, target_pose.position.z))

#####################