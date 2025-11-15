import logging
import time
import rclpy
from threading import Timer
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import os

from rclpy.node import Node
from std_msgs.msg import String, UInt16MultiArray, Float64, Float64MultiArray, MultiArrayDimension, Bool
from geometry_msgs.msg import Pose, Twist, Point, TransformStamped, PoseStamped, Vector3
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from math import cos, sin, degrees, radians, pi, sqrt
from nav_msgs.msg import Odometry, Path
import cflib.crtp
from cflib.utils.power_switch import PowerSwitch
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from multi_agent_pkg.lagrange_multipliers import Sphere

# List of URIs, comment the one you do not want to fly
uris = set()
dron = list()

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


class Agent():
    def __init__(self, parent, node, id, x = None, y = None, z = None, d = None, k=None, point = None, vector = None, a = None, b = None, c = None):
        self.id = id
        self.idn = float(len(parent.agent_list))
        self.distance = False
        self.parent = parent
        self.node = node
        self.disconnect = False
        self.last_error = 0.0
        self.last_iae = 0.0
        self.k = 1.0 # * self.parent.k
        self.pose = Pose()

        if not id.find("line") == -1:
            self.distance_bool = True
            self.d = 0
            self.point = point
            self.vector = vector
            self.mod=pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2)
            self.k = self.k * 4.0
        else:
            if d == None:
                self.x = x
                self.y = y
                self.z = z
                self.node.get_logger().info('Agent: %s' % self.str_())
            else:
                self.d = d
                self.distance = True
                self.node.get_logger().info('Agent: %d %s' % (self.idn, self.str_distance_()))
            if self.id == 'origin' or self.id == 'sphere' or self.id == 'cone' or self.id == 'ellipsoid':
                self.pose.position = point
                self.k = self.k
            self.sub_pose_ = self.node.create_subscription(PoseStamped, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
            if self.parent.config['task']['Onboard'] and self.parent.physical:
                parent.scf.cf.high_level_commander.new_neighbour(self.idn, self.d, self.k)
        if not self.parent.digital_twin:
            self.sub_d_ = self.node.create_subscription(Float64, self.parent.id + '/' + self.id + '/d', self.d_callback, 10)
            self.publisher_data_ = self.node.create_publisher(Float64, self.parent.id + '/' + self.id + '/data', 10)
            self.publisher_order_ = self.node.create_publisher(String, '/' + self.id + '/order', 10)
            self.publisher_error_ = self.node.create_publisher(Float64, self.parent.id + '/' + self.id + '/error', 10)
            # self.publisher_iae_ = self.node.create_publisher(Float64, self.parent.id + '/' + self.id + '/iae', 10)
            self.publisher_marker_ = self.node.create_publisher(Marker, self.parent.id + '/' + self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))
    
    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def d_callback(self, msg):
        self.d = msg.data
        if self.parent.physical:
            self.parent.scf.cf.high_level_commander.update_distance(self.idn, self.d)
        self.node.get_logger().info('Agent: %s: new d: %.2f' % (self.id, self.d))

    def gtpose_callback(self, msg):
        self.pose = msg.pose
        if self.parent.config['task']['Onboard'] and self.parent.config['type'] != 'virtual': #  and not self.disconnect and self.parent.formation and self.parent.physical:
            self.parent.scf.cf.high_level_commander.update_neighbour(self.idn, self.pose.position.x, self.pose.position.y, self.pose.position.z)
        if not self.disconnect and not self.parent.digital_twin:
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
        self.logger.debug('Command: %.3f %.3f' % (self.roll, self.pitch))
        # cf.commander.send_setpoint(self.roll, -self.pitch, 0.0, self.thrust)
        # cf.high_level_commander.update_attitud_cmd(self.roll, self.pitch, self.yaw, self.thrust)
        cf.high_level_commander.update_attituderate_cmd(self.roll, self.pitch, self.yaw, self.thrust)

    def take_off(self, cf):
        self.logger.info('Take off ... ')
        cf.high_level_commander.takeoff(0.8, 1.5)

    def land(self, cf):
        self.logger.info('Take land ... ')
        cf.high_level_commander.land(0.0, 2.0)

class Crazyflie_ROS2():
    def __init__(self, parent, node, link_uri, id, config, scf = None, webots_node=None):
        ## Intialize Physical Crazyflie
        if scf is not None:
            self.scf = scf
            self.scf.uri = link_uri
            self.powerswitch = PowerSwitch(link_uri)
            self.scf.cf.connected.add_callback(self._connected)
            self.scf.cf.disconnected.add_callback(self._disconnected)
            self.scf.cf.connection_failed.add_callback(self._connection_failed)
            self.scf.cf.connection_lost.add_callback(self._connection_lost)
        if webots_node is not None:
            self.robot = webots_node.robot
            timestep = int(self.robot.getBasicTimeStep())
            self.past_time = self.robot.getTime()
            self.first_x_global = 0.0
            self.first_y_global = 0.0
            self.past_x_global = 0
            self.past_y_global = 0
            self.past_z_global = 0
            self.virtualCrazyflie()

        self.parent = parent
        self.node = node
        self.config = config
        self.id = id
        self.tfbr = TransformBroadcaster(self.node)

        ## Read Configuration
        self.control_mode = self.config['control_mode']
        self.node.get_logger().info('%s::Control Mode: %s!' % (self.id, self.control_mode))
        if self.control_mode == 'Gimbal':
            self.iterate_loop = self.node.create_timer(0.01, self.gimbal_iterate)
        self.controller_type = self.config['controller']['type']
        self.node.get_logger().info('%s::Controller Type: %s!' % (self.id, self.controller_type))
        if self.controller_type == 'ipc':
            self.controller_IPC = True
            self.controller_PID = False
            self.eomas = 3.14
        elif self.controller_type == 'pid':
            self.controller_IPC = False
            self.controller_PID = True
        else:
            self.controller_IPC = False
            self.controller_PID = True
        self.communication = (self.config['communication']['type'] == 'Continuous')
        if not self.communication:
            self.threshold = config['communication']['threshold']['co']
        else:
            self.threshold = 0.001

        ## Intialize Variables
        self.state = [10.0, 10.0, 10.0, 10.0, 10.0]
        self.update_gain = True
        self.led_ring = False
        self.ready = False
        self.disconnect = False
        self.swarm_ready = False
        self.digital_twin = self.config['type'] == 'digital_twin'
        self.physical = self.config['type'] == 'physical'
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
        self.mrs_cmd_x = 0.0
        self.mrs_cmd_y = 0.0
        self.mrs_cmd_z = 0.0
        self.Fixed_z = False
        self.N = 1.0

        self.initialize()
    
    def virtualCrazyflie(self):
        # Initialize motors
        self.m1_motor = self.robot.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robot.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)
        # Initialize Sensors
        self.cam = self.robot.getDevice("camera")
        self.cam.disable()
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(timestep)
        self.range_front = self.robot.getDevice("range_front")
        self.range_front.enable(timestep)
        self.range_left = self.robot.getDevice("range_left")
        self.range_left.enable(timestep)
        self.range_back = self.robot.getDevice("range_back")
        self.range_back.enable(timestep)
        self.range_right = self.robot.getDevice("range_right")
        self.range_right.enable(timestep)
        # Intialize Controllers
        
        # Position
        self.z_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.1, 0.01)
        self.x_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 0.5, -0.5, 0.1, 0.01)
        self.y_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 0.5, -0.5, 0.1, 0.01)
        # Velocity
        self.w_controller = PIDController( 25.0, 15.0, 0.0, 0.0, 100, 26.0, -16.0, 0.1, 0.01)
        self.u_controller = PIDController( 15.0,  0.5, 0.0, 0.0, 100, 30.0, -30.0, 0.1, 0.01)
        self.v_controller = PIDController(-15.0,  0.5, 0.0, 0.0, 100, 30.0, -30.0, 0.1, 0.01)
        # Attitude
        self.pitch_controller = PIDController(6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0, 0.1, 0.01)
        self.roll_controller  = PIDController(6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0, 0.1, 0.01)
        # self.yaw_controller   = PIDController(6.0, 1.0, 0.349, 0.0581, 100, 400.0, -400.0, 0.1, 0.01)
        self.yaw_controller   = PIDController(18.86, 0.0, 0.0, 0.0, 100, 400.0, -400.0, 0.1, 0.01)
        # Rate
        self.dpitch_controller = PIDController(250.0, 500.0,   2.5, 0.01, 100, 0.0, -0.0, 0.1, 0.01)
        self.droll_controller  = PIDController(250.0, 500.0,   2.5, 0.01, 100, 0.0, -0.0, 0.1, 0.01)
        self.dyaw_controller   = PIDController(120.0,  16.698, 0.0, 0.00, 100, 0.0, -0.0, 0.1, 0.01)

    def initialize(self):
        self.node.get_logger().info('Connected to %s' % self.id)
        # ROS
        # Publisher
        # POSE3D
        if self.config['local_pose']['enable']:
            self.path_enable = self.config['local_pose']['path']
            if self.path_enable:
                self.path_publisher = self.node.create_publisher(Path, self.id+'/path', 10)
            if self.control_mode == 'None':
                self.sub_pose_ = self.node.create_subscription(PoseStamped, self.id + '/pose', self.newpose_callback, 10)
            elif self.control_mode == 'Gimbal':
                self.publisher_sp_pitch = self.node.create_publisher(Float64, self.id + '/sp_pitch', 10)
                self.publisher_sp_roll = self.node.create_publisher(Float64, self.id + '/sp_roll', 10)
                self.publisher_sp_yaw = self.node.create_publisher(Float64, self.id + '/sp_yaw', 10)
                self.sub_goal_roll_ = self.node.create_subscription(Float64, self.id + '/goal_roll', self.roll_callback, 10)
                self.sub_goal_pitch_ = self.node.create_subscription(Float64, self.id + '/goal_pitch', self.pitch_callback, 10)
                self.sub_goal_yaw_ = self.node.create_subscription(Float64, self.id + '/goal_yaw', self.yaw_callback, 10)
            self.publisher_roll = self.node.create_publisher(Float64, self.id + '/roll', 10)
            self.publisher_pitch = self.node.create_publisher(Float64, self.id + '/pitch', 10)
            self.publisher_yaw = self.node.create_publisher(Float64, self.id + '/yaw', 10)
            pose_name = self.id+'/local_pose'
            if self.digital_twin:
                if not self.physical:
                    pose_name = self.id+'/dt_pose'
                    self.node.create_subscription(PoseStamped, self.id+'/local_pose', self.dt_pose_callback, 1)
                
            self.publisher_pose = self.node.create_publisher(PoseStamped, pose_name, 10)
        # TWIST
        if self.config['local_twist']['enable']:
            self.publisher_twist = self.node.create_publisher(Twist, self.id + '/local_twist', 10)
            
        # DATA ATTITUDE.
        if self.config['data_attitude']['enable']:
            self.publisher_data_attitude = self.node.create_publisher(Float64MultiArray, self.id + '/data_attitude', 10)
            
        # DATA RATE.
        if self.config['data_rate']['enable']:
            self.publisher_data_rate = self.node.create_publisher(Float64MultiArray, self.id + '/data_rate', 10)
            
        # DATA MOTOR.
        if self.config['data_motor']['enable']:
            self.publisher_data_motor = self.node.create_publisher(Float64MultiArray, self.id + '/data_motor', 10)
            
        # MULTIROBOT
        if self.config['mars_data']['enable'] or True:
            self.publisher_goalpose = self.node.create_publisher(PoseStamped, self.id + '/goal_pose', 10)
            self.publisher_mrs_data = self.node.create_publisher(Float64MultiArray, self.id + '/mr_data', 10)
            
        # DATA.
        if self.config['data']['enable']:
            self.publisher_data = self.node.create_publisher(UInt16MultiArray, self.id + '/data', 10)
        if not self.communication:
            self.event_x_ = self.node.create_publisher(Bool, self.id+'/event_x', 10)
            self.event_y_ = self.node.create_publisher(Bool, self.id+'/event_y', 10)
            self.event_z_ = self.node.create_publisher(Bool, self.id+'/event_z', 10)
        # Subscription
        self.sub_goalpose_ = self.node.create_subscription(PoseStamped, self.id+'/goal_pose', self.goalpose_callback, 1)
        self.sub_order_  = self.node.create_subscription(String, self.id+'/order', self.order_callback, 1)
        self.sub_swarmorder_ = self.node.create_subscription(String, 'swarm/order', self.order_callback, 1)
        # self.sub_swarmgoal_ = self.node.create_subscription(PoseStamped, 'swarm/goal_pose', self.swarm_goalpose_callback, 1)
        if not self.control_mode == 'HighLevel':
            self.sub_onboard_ = self.node.create_subscription(Float64MultiArray, self.id + '/onboard_cmd', self.cmd_control_callback, 10)
        # Publisher
        self.laser_publisher = self.node.create_publisher(LaserScan, self.id+'/scan', 10)
        self.swarm_status_publisher = self.node.create_publisher(String, 'swarm/status', 10)
        self.odom_publisher = self.node.create_publisher(Odometry, self.id+'/odom', 10)
        
        # self.msg_laser = LaserScan()
        # self.node.create_timer(0.2, self.publish_laserscan_data)

        # if self.config['task']['enable']:
        #     self.load_formation_params()

        self.node.get_logger().info('%s::inicialize() ok.' % self.id)     

    ##################
    #    Physical    #
    ##################
    def _connected(self, link_uri):
        # POSE3D
        if self.config['local_pose']['enable']:
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
                self.node.get_logger().error('%s. Could not add Stabilizer log config, bad configuration.' % self.id)

        # TWIST
        if self.config['local_twist']['enable']:
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
                self.node.get_logger().error('%s. Could not add Stabilizer log config, bad configuration.' % self.id)
        
        # DATA ATTITUDE.
        if self.config['data_attitude']['enable']:
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
                self.node.get_logger().error('%s. Could not add Stabilizer log config, bad configuration.' % self.id)

        # DATA RATE.
        if self.config['data_rate']['enable']:
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
                self.node.get_logger().error('%s. Could not add Stabilizer log config, bad configuration.' % self.id)

        # DATA MOTOR.
        if self.config['data_motor']['enable']:
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
                self.node.get_logger().error('%s. Could not add Stabilizer log config, bad configuration.' % self.id)

        # MULTIROBOT
        if self.config['mars_data']['enable']:
            if self.config['task']['Onboard']:
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
                    self.node.get_logger().info('Could not start log configuration,'
                        '{} not found in TOC'.format(str(e)))
                except AttributeError:
                    self.node.get_logger().error('%s. Could not add Stabilizer log config, bad configuration.' % self.id)
            
        # DATA.
        if self.config['data']['enable']:
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
                self.node.get_logger().error('%s. Could not add Stabilizer log config, bad configuration.' % self.id)

        self.scf.cf.commander.set_client_xmode(True)
        self.xy_lim = 2.0
        self.cmd_motion_ = CMD_Motion(self.node.get_logger(), xy_lim = self.xy_lim)
        self.load_formation_params()

    def _stab_log_error(self, logconf, msg):
        self.node.get_logger().error('%s. Error when logging %s: %s' % (self.id, logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        if(logconf.name == "Pose"):
            self.pose_callback(data)
            # print('[%d]%s[%s]: %s' % (timestamp, self.id, logconf.name, data))
        elif(logconf.name == "Twist"):
            self.twist_callback(data)
        elif(logconf.name == "Data_attitude"):
            self.dataAttitude_callback(data)
        elif(logconf.name == "Data_rate"):
            self.dataRate_callback(data)
        elif(logconf.name == "Data_multirobot"):
            self.dataMRS_callback(data)
            # print('[%d]%s[%s]: %s' % (timestamp, self.id, logconf.name, data))
        elif(logconf.name == "Data_motor"):
            self.dataMotor_callback(data)
            # print('[%d]%s[%s]: %s' % (timestamp, self.id, logconf.name, data))
        elif(logconf.name == "Data"):
            self.data_callback(data)
        else:
            self.node.get_logger().error('%s. Error: %s: not valid logconf' % (self.id, logconf.name))

    def param_stab_est_callback(self, name, value):
        self.node.get_logger().info('%s. Parameter %s: %s' %(self.id, name, value))

    def _connection_failed(self, link_uri, msg):
        self.node.get_logger().error('%s. Connection to %s failed: %s' % (self.id, link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        self.node.get_logger().error('%s. Connection to %s lost: %s' % (self.id, link_uri, msg))

    def _disconnected(self, link_uri):
        self.node.get_logger().warning('%s. Disconnected from %s' % (self.id, link_uri))
        self.is_connected = False

    ##############
    #    Data    #
    ##############
    def pose_callback(self, data):
        if self.init_pose:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x = data['stateEstimate.x']
            msg.pose.position.y = data['stateEstimate.y']
            msg.pose.position.z = data['stateEstimate.z']
            self.roll = data['stabilizer.roll']
            self.pitch = data['stabilizer.pitch']
            self.yaw = data['stabilizer.yaw']
            q = quaternion_from_euler(0.0, 0.0, 0.0)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            if (abs(self.pitch)>90.0 or abs(self.roll)>90.0) and self._is_flying:
                self.node.get_logger().error('CF%s::Error Angle' % self.scf.cf.link_uri[-2:])
                self.disconnected()


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
                t_base.header.stamp = self.node.get_clock().now().to_msg()
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
                    self.path.header.stamp = self.node.get_clock().now().to_msg()
                    PoseStamp = PoseStamped()
                    PoseStamp.header.frame_id = "map"
                    PoseStamp.pose.position.x = msg.pose.position.x
                    PoseStamp.pose.position.y = msg.pose.position.y
                    PoseStamp.pose.position.z = msg.pose.position.z
                    PoseStamp.pose.orientation.x = msg.pose.orientation.x
                    PoseStamp.pose.orientation.y = msg.pose.orientation.y
                    PoseStamp.pose.orientation.z = msg.pose.orientation.z
                    PoseStamp.pose.orientation.w = msg.pose.orientation.w
                    PoseStamp.header.stamp = self.node.get_clock().now().to_msg()
                    self.path.poses.append(PoseStamp)
                    self.path_publisher.publish(self.path)
        else:
            try:
                if self.scf.cf.param.get_value('deck.bcLighthouse4') == '1' or self.config['positioning'] == 'Intern':
                    msg = PoseStamped()
                    msg.header.frame_id = "map"
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.pose.position.x = data['stateEstimate.x']
                    msg.pose.position.y = data['stateEstimate.y']
                    msg.pose.position.z = data['stateEstimate.z']
                    self.roll = data['stabilizer.roll']
                    self.pitch = data['stabilizer.pitch']
                    self.yaw = data['stabilizer.yaw']
                    q = quaternion_from_euler(0.0, 0.0, 0.0)
                    msg.pose.orientation.x = q[0]
                    msg.pose.orientation.y = q[1]
                    msg.pose.orientation.z = q[2]
                    msg.pose.orientation.w = q[3]
                    t_base = TransformStamped()
                    t_base.header.stamp = self.node.get_clock().now().to_msg()
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
                    self.node.get_logger().info('CF%s::Home pose: %s' % (self.scf.cf.link_uri[-2:], self.cmd_motion_.pose_str_()))
                    msg = String()
                    msg.data = 'init'
                    self.swarm_status_publisher.publish(msg)
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
        msg.layout.data_offset = 0
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = 'data'
        msg.layout.dim[0].size = 5
        msg.layout.dim[0].stride = 1
        self.publisher_data_attitude.publish(msg)

    def dataRate_callback(self, data):
        msg = Float64MultiArray()
        msg.data = {data['controller.rollRate'], data['controller.pitchRate'], data['controller.yawRate'], data['controller.cmd_roll'], data['controller.cmd_pitch'], data['controller.cmd_yaw']}
        msg.layout.data_offset = 0
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = 'data'
        msg.layout.dim[0].size = 6
        msg.layout.dim[0].stride = 1
        self.publisher_data_rate.publish(msg)

    def dataMRS_callback(self, data):
        msg = Float64MultiArray()
        self.mrs_cmd_x = data['multirobot.cmd_x']
        self.mrs_cmd_y = data['multirobot.cmd_y']
        self.mrs_cmd_z = data['multirobot.cmd_z']
        msg.data = {self.mrs_cmd_x, self.mrs_cmd_y, self.mrs_cmd_z}
        # if self.scf.cf.link_uri[-2:] == '07':
        #     self.node.get_logger().info('CF%s::MRS: %.2f %.2f %.2f' % (self.scf.cf.link_uri[-2:], self.mrs_cmd_x, self.mrs_cmd_y, self.mrs_cmd_z))
        msg.layout.data_offset = 0
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = 'data'
        msg.layout.dim[0].size = 3
        msg.layout.dim[0].stride = 1
        self.publisher_mrs_data.publish(msg)
        self.node.get_logger().debug('%s::MARS CMD: %.3f %.3f %.3f' % (self.id, self.mrs_cmd_x, self.mrs_cmd_y, self.mrs_cmd_z))
        if self.config['task']['Onboard'] and self.formation:
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = self.mrs_cmd_x + self.pose.position.x
            target_pose.pose.position.y = self.mrs_cmd_y + self.pose.position.y
            target_pose.pose.position.z = self.mrs_cmd_z + self.pose.position.z
            
            self.publisher_goalpose.publish(target_pose)

    def dataMotor_callback(self, data):
        msg = Float64MultiArray()
        msg.data = {data['controller.cmd_thrust'], data['motor.m1'], data['motor.m2'], data['motor.m3'], data['motor.m4']}
        msg.layout.data_offset = 0
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = 'data'
        msg.layout.dim[0].size = 5
        msg.layout.dim[0].stride = 1
        self.publisher_data_motor.publish(msg)

    def data_callback(self, data):
        msg = UInt16MultiArray()
        msg.data = {data['posEbCtl.Xcount'], data['posEbCtl.Ycount'], data['posEbCtl.Zcount']}
        msg.layout.data_offset = 0
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = 'data'
        msg.layout.dim[0].size = 3
        msg.layout.dim[0].stride = 1
        self.publisher_data.publish(msg)

    def publish_laserscan(self):
        front_range = self.range_front.getValue()/1000.0
        back_range = self.range_back.getValue()/1000.0
        left_range = self.range_left.getValue()/1000.0
        right_range = self.range_right.getValue()/1000.0
        # self.node.get_logger().warn('1: %.3f 2: %.3f 3: %.3f 4: %.3f' % (front_range , back_range, left_range, right_range))

        max_range = 3.49
        if front_range > max_range:
            front_range = float("inf")
        if left_range > max_range:
            left_range = float("inf")
        if right_range > max_range:
            right_range = float("inf")
        if back_range > max_range:
            back_range = float("inf")  

        self.msg_laser = LaserScan()
        self.msg_laser.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        self.msg_laser.header.frame_id = self.id
        self.msg_laser.range_min = 0.1
        self.msg_laser.range_max = max_range
        self.msg_laser.ranges = [back_range, left_range, front_range, right_range, back_range]
        self.msg_laser.angle_min = 0.5 * 2*pi
        self.msg_laser.angle_max =  -0.5 * 2*pi
        self.msg_laser.angle_increment = -1.0*pi/2
        self.laser_publisher.publish(self.msg_laser)

    ###############
    #    Subs    #
    ###############
    def roll_callback(self, msg):
        self.cmd_motion_.roll = msg.data
        self.cmd_motion_.send_offboard_setpoint_(self.scf.cf)

    def pitch_callback(self, msg):
        self.cmd_motion_.pitch = msg.data
        self.cmd_motion_.send_offboard_setpoint_(self.scf.cf)

    def yaw_callback(self, msg):
        self.cmd_motion_.yaw = msg.data
        self.cmd_motion_.send_offboard_setpoint_(self.scf.cf)

    def dt_pose_callback(self, pose):
        self.node.get_logger().debug('TO-DO: DT Pose: X:%f Y:%f' % (pose.pose.position.x,pose.pose.position.y))
        # self.robot.getSelf().getField("translation").setSFVec3f([pose.position.x, pose.position.y, pose.position.z])
        # self.robot.getSelf().getField("rotation").setSFVec3f([0.0, 0.0, 0.0])

    def goalpose_callback(self, pose):
        if self.control_mode == 'HighLevel' and not self.formation:
            self.target_pose = pose
            if self.target_pose.pose.position.z < 0.6:
                self.target_pose.pose.position.z = 0.6

            if self.target_pose.pose.position.z > 2.5:
                self.target_pose.pose.position.z = 2.5
            
            if self.physical:
                self.scf.cf.high_level_commander.go_to_target_pose(self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z)

    def targetpose_callback(self, msg):
        self.cmd_motion_.x = msg.pose.position.x
        self.cmd_motion_.y = msg.pose.position.y
        self.cmd_motion_.z = msg.pose.position.z
        self.node.get_logger().debug('%s::New Target pose: %s' % (self.id, self.cmd_motion_.pose_str_()))
        if self.physical:
            self.scf.cf.high_level_commander.go_to_target_pose(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def cmd_control_callback(self, msg):
        if self.control_mode == 'OffBoard' or self.control_mode == 'Gimbal':
            self.cmd_motion_.roll = msg.data[1]
            self.cmd_motion_.pitch = msg.data[2]
            self.cmd_motion_.yaw = msg.data[3]
            self.cmd_motion_.thrust = int(msg.data[0])
            self.node.get_logger().debug('%s::Command: %s' % (self.id, self.cmd_motion_.str_()))
        else:
            self.node.get_logger().warning('%s::New command control order. Offboard control disabled' % self.id)

    def controllers_params_callback(self, msg):
        self.node.get_logger().info('%s: New %s controller parameters' % (self.id, msg.id))
        if (self.controller_type == 'PID_Continuous'):
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
        elif (self.controller_type == 'PID_EventBased'):
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
        # self.node.get_logger().info('%s::New pose: X:%.2f Y:%.2f Z:%.2f' % (self.id, msg.position.x, msg.position.y, msg.position.z))
        if not self.init_pose:
            self.pose = msg.pose
            self.home = msg.pose
            self.publisher_pose.publish(msg)
            self.scf.cf.extpos.send_extpos(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            self.init_pose = True
            self.cmd_motion_.x = msg.pose.position.x
            self.cmd_motion_.y = msg.pose.position.y
            self.cmd_motion_.z = msg.pose.position.z
            self.node.get_logger().info('CF%s::Init pose: %s' % (self.id, self.cmd_motion_.pose_str_()))
        x = np.array([self.pose.position.x-msg.pose.position.x,self.pose.position.y-msg.pose.position.y,self.pose.position.z-msg.pose.position.z])
        # if (np.linalg.norm(x)>0.005 and np.linalg.norm(x)<0.2):
        self.scf.cf.extpos.send_extpos(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if ((abs(msg.pose.position.x)>self.xy_lim) or (abs(msg.pose.position.y)>self.xy_lim) or (abs(msg.pose.position.z)>2.0)) and self.scf.CONTROL_MODE != 'OffBoard':
            self.control_mode = 'HighLevel'
            self._is_flying = True
            self.node.get_logger().error('CF%s::Out.' % self.id)
            self.gohome()
            t_end = Timer(3, self.descent)
            t_end.start()

    def time_callback(self,msg):
        time = self.node.get_clock().now().to_msg()
        self.node.get_logger().info('%s::Delay: %.2f, %.2f' % (self.id, msg.sec - time.sec, msg.nanosec - time.nanosec))
    
    ##################
    #    Commands    #
    ##################
    def order_callback(self, msg):
        self.node.get_logger().info('%s::Order: "%s"' % (self.id, msg.data))
        if msg.data == 'take_off':
            if self._is_flying:
                self.node.get_logger().warning('%s::Already flying' % self.id)
            else:
                self.take_off()
        elif msg.data == 'land':
            if self._is_flying:
                self.descent()
                self.formation = False
            else:
                self.node.get_logger().warning('%s::In land' % self.id)
        elif msg.data == 'gohome':
            if self._is_flying:
                self.gohome()
            else:
                self.node.get_logger().warning('%s::In land' % self.id)
        elif msg.data == 'formation_run':
            if self.config['task']['enable']:
                self.node.destroy_subscription(self.sub_goalpose_)
                self.formation = True
                if self.physical:
                    if self.led_ring:
                        self.scf.cf.param.set_value('ring.effect', '7')
                    self.scf.cf.high_level_commander.enable_formation()
        elif msg.data == 'formation_stop':
            self.formation = False
            self.sub_goalpose_ = self.node.create_subscription(PoseStamped, self.id+'/goal_pose', self.goalpose_callback, 1)
            if self.physical:
                if self.led_ring:
                    self.scf.cf.param.set_value('ring.effect', '5')
                self.scf.cf.high_level_commander.enable_formation()
        elif msg.data == 'sd_start':
            self.scf.cf.param.set_value('usd.logging', '1')
        elif msg.data == 'sd_stop':
            self.scf.cf.param.set_value('usd.logging', '0')
        elif msg.data == 'disconnect':
            self.disconnected()
        elif msg.data == 'reconfiguration':
            if self.pose.position.z<0.7:
                self.Fixed_z = True
            self.update_gain = True
            self.state = [10.0, 10.0, 10.0, 10.0, 10.0]
            for agent in self.agent_list:
                if agent.id == 'origin':
                    agent.k = 4.0
        elif not msg.data.find("remove") == -1 and self.config['task']['enable']:
            self.remove_agent(msg.data)
        elif not msg.data.find("add") == -1 and self.config['task']['enable']:
            self.add_agent(msg.data)
        elif msg.data == 'rele':
            self.scf.cf.high_level_commander.enable_relay()
        elif msg.data == 'poweroff':
            self.powerswitch.platform_power_down()
        elif msg.data == 'gimbal':
            self.cmd_motion_.roll = 0.0
            self.cmd_motion_.pitch = 0.0
            self.cmd_motion_.yaw = 0.0
            self.gimbal = True
            # level {0 = Rate; 1 = Attitude}, angle {0 = Roll; 1 = Pitch}, cmd, threshold
            # self.scf.cf.high_level_commander.update_relay_params(0, 0, 3000.0, 3.0)   # Roll Rate
            # self.scf.cf.high_level_commander.update_relay_params(1, 0, 5.0, 0.4)      # Roll
            # self.scf.cf.high_level_commander.update_relay_params(0, 1, 8000.0, 6.0)   # Pitch Rate
            self.scf.cf.high_level_commander.update_relay_params(1, 1, 5.0, 0.20)        # Pitch 
            ## Roll Rate
            # self.scf.cf.high_level_commander.update_controller_params(0, 0, 300.0, 100.0, 0.0, 0.0) # Manual
            self.scf.cf.high_level_commander.update_controller_params(0, 0, 250.0, 500.0, 2.5, 0.0) # Serie
            # self.scf.cf.high_level_commander.update_controller_params(0, 0, 0.0, 0.0, 0.0, 0.0) # TEST
            # self.scf.cf.high_level_commander.update_controller_params(0, 0, 166.430, 259.312, 0.0, 0.0) # AMIGO
            # self.scf.cf.high_level_commander.update_controller_params(0, 0, 202.07, 465.37, 0.0, 0.0) # AMIGO init
            # self.scf.cf.high_level_commander.update_controller_params(0, 0, 213.9816, 558.4472, 5.1245, 0.5) # AMIGO PID
            # self.scf.cf.high_level_commander.update_controller_params(0, 0, 232.648, 594.118, 0.0, 0.5) # SIMC
            ## Roll
            # self.scf.cf.high_level_commander.update_controller_params(1, 0, 6.0, 3.0, 0.0, 0.0) # Serie
            # self.scf.cf.high_level_commander.update_controller_params(1, 0, 2.0, 0.0, 0.0, 0.0) # Manual
            # self.scf.cf.high_level_commander.update_controller_params(1, 0, 5.1293, 5.6379, 0.0, 0.1) # AMIGO
            # self.scf.cf.high_level_commander.update_controller_params(1, 0, 6.5948, 12.1417, 0.2239, 0.1) # AMIGO PID
            self.scf.cf.high_level_commander.update_controller_params(1, 0, 5.2053, 7.1637, 0.2364, 0.0) # AMIGO PID
            # self.scf.cf.high_level_commander.update_controller_params(1, 0, 3.7142, 3.4661, 0.0, 0.0) # SIMC
            
            # Pitch Rate
            # self.scf.cf.high_level_commander.update_controller_params(0, 1, 250.0, 500.0, 2.5, 0.0) # Serie
            # self.scf.cf.high_level_commander.update_controller_params(0, 1, 100.0, 300.0, 0.0, 0.0) # Manual
            self.scf.cf.high_level_commander.update_controller_params(0, 1, 276.9316, 188.0486, 0.0, 2.0) # AMIGO PI // OK
            # self.scf.cf.high_level_commander.update_controller_params(0, 1, 356.0549, 404.9760, 19.5652, 0.0) # AMIGO PID
            # self.scf.cf.high_level_commander.update_controller_params(0, 1, 280.5954, 226.3597, 0.0, 0.0) # SIMC
            
            # Pitch
            self.scf.cf.high_level_commander.update_controller_params(1, 1, 1.5, 0.0, 0.0, 0.0) # Manual
            # self.scf.cf.high_level_commander.update_controller_params(1, 1, 6.0, 3.0, 0.0, 0.0) # Serie
            # self.scf.cf.high_level_commander.update_controller_params(1, 1, 1.3295, 0.5088, 0.0, 0.0) # AMIGO PI
            # self.scf.cf.high_level_commander.update_controller_params(1, 1, 1.7094, 1.0958, 0.1667, 0.0) # AMIGO PID
            # self.scf.cf.high_level_commander.update_controller_params(1, 1, 1.8753, 1.1869, 0.0, 0.0) # SIMC

            # self.node.get_logger().warn('Relay:: cmd:10000  th:1')
            self.scf.cf.param.set_value('stabilizer.estimator', '1')
            self.scf.cf.param.set_value('kalman.resetEstimation', '1')
            self.scf.cf.param.set_value('kalman.resetEstimation', '0')
            self.scf.cf.param.set_value('stabilizer.controller', '5')
            
            self.scf.cf.param.set_value('flightmode.stabModeRoll', '1')
            self.scf.cf.param.set_value('flightmode.stabModePitch', '1')
            self.scf.cf.param.set_value('flightmode.stabModeYaw', '1')
            
            # AMIGO
            # self.scf.cf.param.set_value('pid_rate.pitch_kp', 581.9545)
            # self.scf.cf.param.set_value('pid_rate.pitch_ki', 634.1221)
            # SIMC
            # self.scf.cf.param.set_value('pid_rate.pitch_kp', 424.1390)
            # self.scf.cf.param.set_value('pid_rate.pitch_ki', 394.9337)
            # Z-N
            # self.scf.cf.param.set_value('pid_rate.pitch_kp', 1044.7)
            # self.scf.cf.param.set_value('pid_rate.pitch_ki', 4766.9)
            # self.scf.cf.param.set_value('pid_rate.pitch_kd', 0.0)

            # self.scf.cf.param.set_value('pid_rate.yaw_kp', 0.0)
            # self.scf.cf.param.set_value('pid_rate.yaw_ki', 0.0)
            # self.scf.cf.param.set_value('pid_rate.yaw_kd', 0.0)
            self.scf.cf.param.set_value('pid_attitude.yaw_kp', 0.0)
            self.scf.cf.param.set_value('pid_attitude.yaw_ki', 0.0)
            self.scf.cf.param.set_value('pid_attitude.yaw_kd', 0.0)

            self.scf.cf.commander.send_setpoint(0, 0, 0, 0)
            
            self.cmd_motion_.thrust = 3001
        else:
            self.node.get_logger().error('%s::"%s": Unknown order' % (self.id, msg.data))
    
    def take_off(self):
        self.node.get_logger().info('%s::Take Off...' % self.id)
        if self.led_ring:
            self.scf.cf.param.set_value('ring.effect', '7')
            self.scf.cf.param.set_value('ring.solidRed', '100')
            self.scf.cf.param.set_value('ring.solidGreen', '0')
            self.scf.cf.param.set_value('ring.solidBlue', '0')

        self.target_pose.pose.position.z = 0.8
        if self.physical:
            self.scf.cf.high_level_commander.takeoff(self.target_pose.pose.position.z, 2.0)
        self._is_flying = True
        self.t_ready = Timer(2, self._ready)
        self.t_ready.start()

    def _ready(self):
        self.node.get_logger().info('%s::Ready!!' % self.id)
        if self.led_ring:
            self.scf.cf.param.set_value('ring.effect', '5')
            self.scf.cf.param.set_value('ring.solidRed', '0')
            self.scf.cf.param.set_value('ring.solidGreen', '0')
            self.scf.cf.param.set_value('ring.solidBlue', '100')
        self.ready = True

    def gohome(self):
        self.node.get_logger().info('%s::Go Home.' % self.id)
        if self.physical:
            self.scf.cf.high_level_commander.go_to_target_pose(self.home.position.x, self.home.position.y, 0.7)
    
    def descent(self):
        if self._is_flying:
            self.node.get_logger().info('%s::Descent.' % self.id)
            self.target_pose.pose.position.z = 0.07
            if self.physical:
                self.scf.cf.param.set_value('stabilizer.controller', '1')
                self.cmd_motion_.land(self.scf.cf)
                self.cmd_motion_.send_pose_data_(self.scf.cf)
                self.scf.cf.high_level_commander.land(0.0, 2.0)
            if self.led_ring:
                self.scf.cf.param.set_value('ring.effect', '7')
                self.scf.cf.param.set_value('ring.solidRed', '100')
                self.scf.cf.param.set_value('ring.solidGreen', '0')
                self.scf.cf.param.set_value('ring.solidBlue', '0')
            self.t_desc = Timer(2, self.take_land)
            self._is_flying = False
            self.ready = False
            self.t_desc.start()
        else:
            self.node.get_logger().error('%s::In land' % self.id)

    def take_land(self):
        self.node.get_logger().info('%s::Take Land.' % self.id)
        self.target_pose.pose.position.z = 0.0
        if self.physical:
            self.scf.cf.commander.send_setpoint(0.0, 0.0, 0, 0)
            self.scf.cf.commander.send_stop_setpoint()
        if self.led_ring:
            self.scf.cf.param.set_value('ring.effect', '2')
        self.init_pose = False

    def swarm_goalpose_callback(self, msg):
        if not self.centroid_leader:
            self.node.get_logger().info('Formation Control::Leader-> Centroid.')
        self.centroid_leader = True
        self.leader_cmd = msg

    def gimbal_iterate(self):
        msg = Float64()
        # self.node.get_logger().info('CF:::SP_Pitch: %.2f, Pitch: %.2f' % (self.cmd_motion_.pitch, self.pitch))
        if self.gimbal:
            # self.node.get_logger().info('SetPoint:::Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Thrust: %d' % (self.cmd_motion_.roll, self.cmd_motion_.pitch, self.cmd_motion_.yaw, self.cmd_motion_.thrust))
            # self.pitch_controller.error[0] = self.sp_pitch - self.pitch
            # self.cmd_motion_.pitch = self.pitch_controller.update(0.01)
            # self.cmd_motion_.pitch = self.sp_pitch
            # self.cmd_motion_.pitch = self.pitch_controller.rele_update(0.01)
            # self.scf.cf.extpos.send_extpos(0.0, 0.0, 0.7)
            try:
                msg.data = self.cmd_motion_.pitch
                self.publisher_sp_pitch.publish(msg)
                msg.data = self.cmd_motion_.roll
                self.publisher_sp_roll.publish(msg)
                msg.data = self.cmd_motion_.yaw
                self.publisher_sp_yaw.publish(msg)
            except:
                pass
            # self.cmd_motion_.send_offboard_setpoint_(self.scf.cf)
    
    def disconnected(self):
        self.formation = False
        if self.physical:
            if self.led_ring:
                self.scf.cf.param.set_value('ring.effect', '5')
            self.scf.cf.param.set_value('stabilizer.controller', '1')
        self.descent()
        for agent in self.agent_list:
            agent.disconnect = True
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = self.node.get_clock().now().to_msg()
            line.id = 1
            line.type = 5
            line.action = 0
            line.scale.x = 0.01
            line.scale.y = 0.01
            line.scale.z = 0.01
            agent.publisher_marker_.publish(line)
            agent.node.destroy_publisher(agent.publisher_marker_)
            msg = String()
            msg.data = 'remove_'+self.id
            agent.publisher_order_.publish(msg)

    def add_agent(self, data):
        aux = data.split('_')
        robot = Agent(self, self.parent, aux[1], d = float(aux[2]))
        self.agent_list.append(robot)
        self.N = self.N + 1
    
    def remove_agent(self, data):
        aux = data.split('_')
        j = 0
        for agent in self.agent_list:
            if agent.id == aux[1]:
                if self.physical:
                    agent.parent.scf.cf.high_level_commander.remove_neighbour(agent.idn)
                agent.disconnect = True
                line = Marker()
                line.header.frame_id = 'map'
                line.header.stamp = self.node.get_clock().now().to_msg()
                line.id = 1
                line.type = 5
                line.action = 0
                line.scale.x = 0.01
                line.scale.y = 0.01
                line.scale.z = 0.01
                agent.publisher_marker_.publish(line)
                agent.node.destroy_publisher(agent.publisher_marker_)
                self.agent_list.pop(j)
                self.N = self.N - 1
            else:
                j += 1


    ###############
    #    Tasks    #
    ###############
    def load_formation_params(self):
        if self.config['task']['enable']:
            self.N = 0
            # self.node.destroy_subscription(self.sub_goalpose_)
            self.publisher_global_error_ = self.node.create_publisher(Float64, self.id + '/global_error', 10)
            self.node.get_logger().info('Task %s by %s' % (self.config['task']['type'], self.config['task']['role']))
            self.agent_list = list()
            self.controller = self.config['task']['controller']
            self.controller_type = self.controller['type']
            self.k = self.controller['gain']
            self.ul = self.controller['upperLimit']
            self.ll = self.controller['lowerLimit']
            self.continuous = self.controller['protocol'] == 'Continuous'
            aux = self.config['task']['relationship']
            if aux == 'empty':
                self.relationship = 'empty'
            else:
                self.relationship = aux.split(', ')
                if self.config['task']['type'] == 'distance':
                    self.task_period = self.config['task']['T']/1000
                    if self.config['task']['Onboard']:
                        self.timer_task = self.node.create_timer(self.task_period, self.task_formation_info)
                    else:
                        self.timer_task = self.node.create_timer(self.task_period, self.task_formation_distance)
                    for rel in self.relationship:
                        aux = rel.split('_')
                        robot = Agent(self, self.node, aux[0], d = float(aux[1]), k = self.k)
                        self.agent_list.append(robot)
                        self.N = self.N + 1
                elif self.config['task']['type'] == 'relative_pose':
                    self.timer_task = self.node.create_timer(self.config['task']['T']/1000, self.task_formation_pose)
                    for rel in self.relationship:
                        aux = rel.split('_')
                        rel_pose = aux[1].split('/')
                        robot = Agent(self, self.node, aux[0], x = float(rel_pose[0]), y = float(rel_pose[1]), z = float(rel_pose[2]))
                        self.agent_list.append(robot)
    
    def task_formation_distance(self):
        if self.formation:
            msg_error = Float64()
            msg_error.data = 0.0
            dx = dy = dz = 0
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            for agent in self.agent_list:
                error_x = self.pose.position.x - agent.pose.position.x
                error_y = self.pose.position.y - agent.pose.position.y
                error_z = self.pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)

                dx += - agent.k * (distance - pow(agent.d,2)) * error_x
                dy += - agent.k * (distance - pow(agent.d,2)) * error_y
                dz += - agent.k * (distance - pow(agent.d,2)) * error_z
                
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

            # dz = dz * 0.5
            msg = Float64MultiArray()
            msg.data = [round(dx,3), round(dy,3), round(dz,3), 5.0]
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 4
            msg.layout.dim[0].stride = 1
            self.publisher_mrs_data.publish(msg)

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
            
            target_pose.pose.position.x = self.pose.position.x + dx
            target_pose.pose.position.y = self.pose.position.y + dy
            target_pose.pose.position.z = self.pose.position.z + dz

            # TO-DO: Delta_{dx,dy,dz}
            delta = sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
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
            if self.led_ring:
                self.scf.cf.param.set_value('ring.solidBlue', '0')
                if delta>0.1:
                    self.scf.cf.param.set_value('ring.solidRed', '100')
                    self.scf.cf.param.set_value('ring.solidGreen', '0')
                elif delta > 0.05:
                    self.scf.cf.param.set_value('ring.solidRed', '50')
                    self.scf.cf.param.set_value('ring.solidGreen', '50')
                else:
                    self.scf.cf.param.set_value('ring.solidRed', '0')
                    self.scf.cf.param.set_value('ring.solidGreen', '100')

            if target_pose.pose.position.z < 0.8:
                target_pose.pose.position.z = 0.8

            if target_pose.pose.position.z > 2.0:
                target_pose.pose.position.z = 2.0
            
            # if self.id == 'dron01' or self.id == 'dron02' or self.id == 'dron03' or self.id == 'dron04': # or self.id == 'dron05':
            #     target_pose.pose.position.z = 0.8
            
            if self.id == 'dron01':
                self.node.get_logger().info('CF:%s - Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (self.id, self.pose.position.x, target_pose.pose.position.x, self.pose.position.y, target_pose.pose.position.y, self.pose.position.z, target_pose.pose.position.z)) 
            

            self.targetpose_callback(target_pose)
            
            self.publisher_goalpose.publish(target_pose)
            self.publisher_global_error_.publish(msg_error)

    def task_formation_info(self):
        if self.formation:
            msg_error = Float64()
            msg_error.data = 0.0
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
            self.publisher_global_error_.publish(msg_error)

            delta = sqrt(pow(self.mrs_cmd_x,2)+pow(self.mrs_cmd_y,2)+pow(self.mrs_cmd_z,2))
            if self.led_ring:
                self.scf.cf.param.set_value('ring.solidBlue', '0')
                if delta>0.1:
                    self.scf.cf.param.set_value('ring.solidRed', '100')
                    self.scf.cf.param.set_value('ring.solidGreen', '0')
                elif delta > 0.05:
                    self.scf.cf.param.set_value('ring.solidRed', '50')
                    self.scf.cf.param.set_value('ring.solidGreen', '50')
                else:
                    self.scf.cf.param.set_value('ring.solidRed', '0')
                    self.scf.cf.param.set_value('ring.solidGreen', '100')

            # self.node.get_logger().info('CF:%s - Formation: X: %.2f Y: %.2f Z: %.2f' % (self.id, self.mrs_cmd_x, self.mrs_cmd_y, self.mrs_cmd_z)) 


