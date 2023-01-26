import logging
import time
import rclpy
from threading import Timer
import numpy as np
import yaml

from rclpy.node import Node
from std_msgs.msg import String, UInt16, UInt16MultiArray, Float64, Float64MultiArray
from geometry_msgs.msg import Pose, Twist, Point, TransformStamped
from visualization_msgs.msg import Marker
from uned_crazyflie_config.msg import Pidcontroller
from tf2_ros import TransformBroadcaster
from math import cos, sin, degrees, radians, pi, sqrt

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# List of URIs, comment the one you do not want to fly
uris = set()
dron = list()

class Agent():
    def __init__(self, parent, node, id, x = None, y = None, z = None, d = None):
        self.id = id
        self.parent = parent
        self.node = node
        if d == None:
            self.x = x
            self.y = y
            self.z = z
            self.node.get_logger().info('Agent: %s' % self.str_())
        else:
            self.d = d
            self.node.get_logger().info('Agent: %s' % self.str_distance_())
        self.pose = Pose()
        self.sub_pose_ = self.node.create_subscription(Pose, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        self.publisher_data_ = self.node.create_publisher(Float64, self.parent.id + '/' + self.id + '/data', 10)
        self.publisher_marker_ = self.node.create_publisher(Marker, self.parent.id + '/' + self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))
    
    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def gtpose_callback(self, msg):
        self.pose = msg
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
        self.parent.distance_formation_bool = True

        distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
    
        line.header.frame_id = 'map'
        line.header.stamp = self.node.get_clock().now().to_msg()
        line.id = 1
        line.type = 5
        line.action = 0
        line.scale.x = 0.01
        line.scale.y = 0.01
        line.scale.z = 0.01

        if abs(distance - self.d) > 0.05:
            line.color.r = 1.0
        else:
            if abs(distance - self.d) > 0.025:
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
        cf.high_level_commander.land(0.0, 1.5)

######################
## CF Logging Class ##
######################
class Crazyflie_ROS2():
    def __init__(self, parent, scf, link_uri, config):
        self.scf = scf
        self.parent = parent
        self.config = config
        self.id = self.config['name']
        # self.node = rclpy.create_node(self.id+'_driver', namespace='/my_ns', use_global_arguments=False)
        # self.node = rclpy.create_node(self.id+'_driver', use_global_arguments=False)
        self.ready = False
        self.swarm_ready = False
        self.distance_formation_bool = False
        self.pose = Pose()
        self.home = Pose()
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
            self.threshold = config['communication']['threshold']['value']
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
        if not self.scf.CONTROL_MODE == 'None':
            self.parent.create_subscription(Pose, self.id + '/pose', self.newpose_callback, 10)
        if self.scf.CONTROL_MODE == 'HighLevel':
            self.parent.create_subscription(Pose, self.id + '/goal_pose', self.goalpose_callback, 10)
        else:
            self.parent.create_subscription(Float64MultiArray, self.id + '/onboard_cmd', self.cmd_control_callback, 10)
        self.parent.create_subscription(Pidcontroller, self.id + '/controllers_params', self.controllers_params_callback, 10)
        
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
        self.cmd_motion_ = CMD_Motion(self.parent.get_logger(), xy_lim = self.xy_lim)
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
        self.parent.destroy_node()

    def take_off(self):
        self.parent.get_logger().info('CF%s::Take Off.' % self.scf.cf.link_uri[-2:])
        # self.cmd_motion_.take_off(self.scf.cf)
        self.cmd_motion_.z = 0.75
        self.cmd_motion_.send_pose_data_(self.scf.cf)
        self._is_flying = True
        self.t_ready = Timer(3, self._ready)
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
        self.cmd_motion_.z = 0.07
        self.cmd_motion_.land(self.scf.cf)
        # self.cmd_motion_.send_pose_data_(self.scf.cf)
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

            delta = np.array([self.pose.position.x-msg.position.x,self.pose.position.y-msg.position.y,self.pose.position.z-msg.position.z])
            
            if self.communication or np.linalg.norm(delta)>self.threshold:
                self.pose = msg
                self.publisher_pose.publish(msg)
                t_base = TransformStamped()
                t_base.header.stamp = self.parent.get_clock().now().to_msg()
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
        # self.parent.get_logger().info('CF%s::New pose: X:%.2f Y:%.2f Z:%.2f' % (self.scf.cf.link_uri[-2:], msg.position.x, msg.position.y, msg.position.z))
        if not self.init_pose:
            self.parent.get_logger().info('CF%s::Test.' % self.scf.cf.link_uri[-2:])
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
            self.cmd_motion_.ckeck_pose()
            self.cmd_motion_.send_pose_data_(self.scf.cf)
            self.parent.get_logger().debug('CF%s::New Goal pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))

    ###############
    #    Tasks    #
    ###############
    def task_formation_distance(self):
        if self.ready and self.swarm_ready and self.distance_formation_bool:
            self.distance_formation_bool = False
            dx = dy = dz = 0
            target_pose = Pose()
            for agent in self.agent_list:
                # self.parent.get_logger().info('Agent: %s' % agent.id)
                error_x = self.pose.position.x - agent.pose.position.x
                error_y = self.pose.position.y - agent.pose.position.y
                error_z = 0 # self.pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                dx += (pow(agent.d,2) - distance) * error_x
                dy += (pow(agent.d,2) - distance) * error_y
                dz += (pow(agent.d,2) - distance) * error_z

                msg_data = Float64()
                msg_data.data = agent.d - sqrt(distance)
                agent.publisher_data_.publish(msg_data)

            delta = sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))

            if dx > 0.32:
                dx = 0.32
            if dx < -0.32:
                dx = -0.32
            if dy > 0.32:
                dy = 0.32
            if dy < -0.32:
                dy = -0.32

            target_pose.position.x = self.pose.position.x + dx/4
            target_pose.position.y = self.pose.position.y + dy/4
            target_pose.position.z = 0.8 # self.pose.position.z # + dz/4
            
            self.goalpose_callback(target_pose)

            self.parent.get_logger().debug('Distance: %.4f eX: %.2f eY: %.2f eZ: %.2f' % (sqrt(distance), error_x, error_y, error_z))
            self.parent.get_logger().info('Delta: %.4f X: %.2f Y: %.2f Z: %.2f' % (delta, dx, dy, dz))
            self.parent.get_logger().info('Target: X: %.2f Y: %.2f Z: %.2f' % (target_pose.position.x, target_pose.position.y, target_pose.position.z))


#####################
## CF Swarm Class  ##
#####################
class CFSwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('first_uri', 'radio://0/80/2M/E7E7E7E701')
        self.declare_parameter('n', 1)
        self.declare_parameter('config', 'file_path.yaml')

        # Publisher
        self.publisher_status_ = self.create_publisher(String,'/swarm/status', 10)
        # Subscription
        self.sub_order = self.create_subscription(String, '/swarm/order', self.order_callback, 10)
        self.sub_pose_ = self.create_subscription(Pose, '/swarm/pose', self.newpose_callback, 10)
        self.sub_goal_pose_ = self.create_subscription(Pose, '/swarm/goal_pose', self.goalpose_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('SwarmDriver::inicialize() ok.')
        self.pose = Pose()
        # Read Params
        dron_id = self.get_parameter('first_uri').get_parameter_value().string_value
        n = self.get_parameter('n').get_parameter_value().integer_value
        config_file = self.get_parameter('config').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
            
        # Define crazyflie URIs
        id_address = dron_id[-10:]
        id_base = dron_id[:16]
        id_address_int = int(id_address, 16)
        for i in range(int(n),0,-1):
            cf_str = id_base + hex(id_address_int+i-1)[-10:].upper()
            uris.add(cf_str)
            self.get_logger().info('Crazyflie %d URI: %s!' % (i, cf_str))

        # logging.basicConfig(level=logging.DEBUG)
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        self.cf_swarm = Swarm(uris, factory=factory)
        i = 0
        for uri in uris:
            id = 'dron' + uri[-2:]
            config = documents[id]
            cf = Crazyflie_ROS2(self, self.cf_swarm._cfs[uri], uri, config)
            dron.append(cf)
            while not cf.scf.cf.param.is_updated:
                time.sleep(1.0)
            self.get_logger().warning('Parameters downloaded for %s' % cf.scf.cf.link_uri)
            i += 1

        self.cf_swarm.parallel_safe(self.update_params)

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
        if (scf.CONTROLLER_TYPE == 'EventBased'):
            scf.cf.param.set_value('controller.eventBased', '1')

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
            cf.cmd_motion_.ckeck_pose()
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
