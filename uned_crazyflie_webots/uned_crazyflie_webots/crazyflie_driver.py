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
import yaml

from std_msgs.msg import (
    String, Bool, Float64, Float64MultiArray, MultiArrayDimension, UInt16MultiArray)
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker

from math import atan2, cos, sin, degrees, pi, sqrt
import numpy as np
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from uned_crazyflie_driver.agent import Agent
from multi_agent_pkg.lagrange_multipliers import Sphere, Cone, Ellipsoid

from uned_crazyflie_driver.pid_controller import PIDController

# Change this path to your crazyflie-firmware folder
# sys.path.append('/home/kiko/Code/crazyflie-firmware')
# import cffirmware


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

        # self.crazyflie = Crazyflie_ROS2(self, self.node, self.id, self.id, self.config,
        # webots_node=webots_node)

        # Intialize Variables
        self.state = [10.0, 10.0, 10.0, 10.0, 10.0]
        self.update_gain = True
        self.ready = False
        self.disconnect = False
        self.swarm_ready = False
        self.digital_twin = self.config['type'] == 'digital_twin'
        self.past_time = self.robot.getTime()
        self.publisher_twist_enable = False
        self.publisher_data_attitude_enable = False
        self.publisher_data_rate_enable = False
        self.publisher_data_motor_enable = False
        self.publisher_mrs_data_enable = False
        self.publisher_data_enable = False
        self.controller_type = 'gradient'
        # Rele
        self.rele_p = False
        self.rele_s = False
        self.rele_a = False
        self.rele_r = False
        self.rele_x = False
        self.rele_y = False
        self.rele_z = False
        self.rele_cmd_eq = 0.0
        self.relay_level = False
        # Position level
        self.relay_p_cmd = 0.2
        self.relay_p_threshold = 0.02
        # Speed level
        self.relay_s_cmd = 4.0
        self.relay_s_threshold = 0.05
        # Attitude level
        self.relay_a_cmd = 15.0
        self.relay_a_threshold = 0.2
        # Rate level
        self.relay_r_cmd = 0.0
        self.relay_r_threshold = 0.0

        self.target_twist = Twist()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.0
        self.pose = Pose()
        self.home = Pose()
        self.path = Path()
        self.path.header.frame_id = "map"
        self.first_x_global = 0.0
        self.first_y_global = 0.0
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_z_global = 0

        self._is_flying = False
        self.init_pose = False
        self.formation = False
        self.Fixed_z = False
        self.distance_formation_bool_update = True
        self.N = 1.0

        self.centroid_leader = False
        self.leader_cmd = PoseStamped()
        self.leader_cmd.header.frame_id = "map"
        self.last_pose = Pose()
        self.trigger_ai = 0.01
        self.trigger_co = 0.1
        self.trigger_last_signal = 0.0

        # Init Config Params
        '''
        self.led = self.robot.getDevice('status')
        if self.id == 'dron01' or self.id == 'dron05' or self.id == 'dron10':
            self.led.set(1)
        else:
            self.led.set(0)

        if self.id == 'dron01':
            self.led.set(2)
        '''
        if "control_mode" in self.config:
            self.control_mode = self.config['control_mode']
        else:
            self.control_mode = 'HighLevel'
        self.node.get_logger().info(
            'Crazyflie %s::Control Mode: %s!' %
            (self.id, self.control_mode))
        if "positioning" in self.config:
            self.positioning = self.config['positioning']
        else:
            self.positioning = 'Intern'

        self.continuous = True

        if "controller" in self.config:
            self.CONTROLLER_TYPE = self.config['controller']['type']
            if self.config['controller']['type'] == 'ipc':
                self.controller_IPC = True
                self.controller_PID = False
                self.eomas = 3.14
            elif self.config['controller']['type'] == 'pid':
                self.controller_IPC = False
                self.controller_PID = True
            else:
                self.controller_IPC = False
                self.controller_PID = True
        else:
            self.CONTROLLER_TYPE = 'pid'
            self.controller_IPC = False
            self.controller_PID = True
        self.node.get_logger().info(
            'Crazyflie %s::Controller Type: %s!' %
            (self.id, self.CONTROLLER_TYPE))

        if "type" in self.config:
            self.physical = self.config['type'] == 'physical'
        else:
            self.physical = False

        if "communication" in self.config:
            self.communication = self.config['communication']['type'] == 'Continuous'
            if not self.communication:
                self.threshold = self.config['communication']['threshold']['co']
            else:
                self.threshold = 0.001
        else:
            self.communication = True
            self.threshold = 0.001

        if "local_pose" in self.config:
            self.config_local_pose = self.config['local_pose']['enable']
        else:
            self.config_local_pose = False

        if "path" in self.config['local_pose']:
            self.path_enable = self.config['local_pose']['path']
        else:
            self.path_enable = False

        if "local_twist" in self.config:
            self.publisher_twist_enable = self.config['local_twist']['enable']
        else:
            self.publisher_twist_enable = False

        if "data_attitude" in self.config:
            self.publisher_data_attitude_enable = self.config['data_attitude']['enable']
        else:
            self.publisher_data_attitude_enable = False

        if "data_rate" in self.config:
            self.publisher_data_rate_enable = self.config['data_rate']['enable']
        else:
            self.publisher_data_rate_enable = False

        if "data_motor" in self.config:
            self.publisher_data_motor_enable = self.config['data_motor']['enable']
        else:
            self.publisher_data_motor_enable = False

        if "mars_data" in self.config:
            self.publisher_mrs_data_enable = self.config['mars_data']['enable']
        else:
            self.publisher_mrs_data_enable = False

        if "data" in self.config:
            self.publisher_data_enable = self.config['data']['enable']
        else:
            self.publisher_data_enable = False

        if "task" in self.config:
            self.task_config = self.config['task']['enable']
            self.task_onboard = self.config['task']['Onboard']
        else:
            self.task_config = False
            self.task_onboard = True

        # Intialize Crazyflie configuration
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
        self.w_controller = PIDController(25.0, 15.0, 0.0, 0.0, 100, 26.0, -16.0, 0.1, 0.01)
        self.u_controller = PIDController(15.0, 0.5, 0.0, 0.0, 100, 30.0, -30.0, 0.1, 0.01)
        self.v_controller = PIDController(-15.0, 0.5, 0.0, 0.0, 100, 30.0, -30.0, 0.1, 0.01)
        # Attitude
        self.pitch_controller = PIDController(6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0, 0.1, 0.01)
        self.roll_controller = PIDController(6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0, 0.1, 0.01)
        # self.yaw_controller   = PIDController(6.0, 1.0, 0.349, 0.0581, 100, 400.0, -400.0, 0.1,
        # 0.01)
        self.yaw_controller = PIDController(18.86, 0.0, 0.0, 0.0, 100, 400.0, -400.0, 0.1, 0.01)
        # Rate
        self.dpitch_controller = PIDController(250.0, 500.0, 2.5, 0.01, 100, 0.0, -0.0, 0.1, 0.01)
        self.droll_controller = PIDController(250.0, 500.0, 2.5, 0.01, 100, 0.0, -0.0, 0.1, 0.01)
        self.dyaw_controller = PIDController(120.0, 16.698, 0.0, 0.00, 100, 0.0, -0.0, 0.1, 0.01)

        # cffirmware.controllerPidInit()

        self.initialize()

    def initialize(self):
        self.node.get_logger().info('Connected to Webots -> Crazyflie %s' % self.id)

        # POSE3D
        if self.config_local_pose:
            if self.path_enable:
                self.path_publisher = self.node.create_publisher(Path, self.id + '/path', 10)
            if self.digital_twin:
                pose_name = self.id + '/dt_pose'
                self.node.create_subscription(
                    PoseStamped, self.id + '/pose_dt', self.dt_pose_callback, 1)
            else:
                pose_name = self.id + '/local_pose'
            self.pose_publisher = self.node.create_publisher(PoseStamped, pose_name, 10)
            self.pose_publisher_gt = self.node.create_publisher(PoseStamped, pose_name + '_gt', 10)
        # TWIST
        if self.publisher_twist_enable:
            self.publisher_twist = self.node.create_publisher(Twist, self.id + '/local_twist', 10)

        # DATA ATTITUDE.
        if self.publisher_data_attitude_enable:
            self.publisher_data_attitude = self.node.create_publisher(
                Float64MultiArray, self.id + '/data_attitude', 10)

        # DATA RATE.
        if self.publisher_data_rate_enable:
            self.publisher_data_rate = self.node.create_publisher(
                Float64MultiArray, self.id + '/data_rate', 10)

        # DATA MOTOR.
        if self.publisher_data_motor_enable:
            self.publisher_data_motor = self.node.create_publisher(
                Float64MultiArray, self.id + '/data_motor', 10)

        # MULTIROBOT
        if self.publisher_mrs_data_enable:
            self.publisher_mrs_data = self.node.create_publisher(
                Float64MultiArray, self.id + '/mr_data', 10)
            self.publisher_mrs_data_gt = self.node.create_publisher(
                Float64MultiArray, self.id + '/mr_data_gt', 10)
            self.publisher_mrs_data_mod = self.node.create_publisher(
                Float64, self.id + '/mr_data_mod', 10)
            self.publisher_mrs_data_gt_mod = self.node.create_publisher(
                Float64, self.id + '/mr_data_gt_mod', 10)

        # DATA.
        if self.publisher_data_enable:
            self.publisher_data = self.node.create_publisher(
                UInt16MultiArray, self.id + '/data', 10)
        if not self.communication:
            self.event_x_ = self.node.create_publisher(Bool, self.id + '/event_x', 10)
            self.event_y_ = self.node.create_publisher(Bool, self.id + '/event_y', 10)
            self.event_z_ = self.node.create_publisher(Bool, self.id + '/event_z', 10)
        # Subscription
        self.node.create_subscription(Twist, self.id + '/cmd_vel', self.cmd_vel_callback, 1)
        self.sub_goalpose = self.node.create_subscription(
            PoseStamped, self.id + '/goal_pose', self.goal_pose_callback, 1)
        self.node.create_subscription(
            PoseStamped,
            self.id +
            '/target_pose',
            self.goal_pose_callback,
            1)
        self.node.create_subscription(String, self.id + '/order', self.order_callback, 1)
        self.node.create_subscription(String, 'swarm/order', self.order_callback, 1)
        self.node.create_subscription(PoseStamped, 'swarm/goal_pose',
                                      self.swarm_goalpose_callback, 1)
        # Publisher
        self.laser_publisher = self.node.create_publisher(LaserScan, self.id + '/scan', 10)
        self.swarm_status_publisher = self.node.create_publisher(String, 'swarm/status', 10)
        self.odom_publisher = self.node.create_publisher(Odometry, self.id + '/odom', 10)

        self.tfbr = TransformBroadcaster(self.node)
        self.msg_laser = LaserScan()
        self.node.create_timer(0.2, self.publish_laserscan_data)

        if self.task_config or self.task_onboard:
            self.load_formation_params()

        self.node.get_logger().info('Webots_Node::inicialize() ok. %s' % (str(self.id)))
        msg = String()
        msg.data = 'init'
        self.swarm_status_publisher.publish(msg)

    def publish_laserscan_data(self):
        front_range = self.range_front.getValue() / 1000.0
        back_range = self.range_back.getValue() / 1000.0
        left_range = self.range_left.getValue() / 1000.0
        right_range = self.range_right.getValue() / 1000.0
        # self.node.get_logger().warn('1: %.3f 2: %.3f 3: %.3f 4: %.3f' % (front_range ,
        # back_range, left_range, right_range))

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
        self.msg_laser.angle_min = 0.5 * 2 * pi
        self.msg_laser.angle_max = -0.5 * 2 * pi
        self.msg_laser.angle_increment = -1.0 * pi / 2
        self.laser_publisher.publish(self.msg_laser)

    def dt_pose_callback(self, pose):
        self.node.get_logger().debug(
            'TO-DO: DT Pose: X:%f Y:%f' %
            (pose.pose.position.x, pose.pose.position.y))
        # self.robot.getSelf().getField("translation").setSFVec3f([pose.position.x,
        # pose.position.y, pose.position.z])
        # self.robot.getSelf().getField("rotation").setSFVec3f([0.0, 0.0, 0.0])

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def goal_pose_callback(self, pose):
        self.target_pose = pose
        if self.target_pose.pose.position.z < 0.6:
            self.target_pose.pose.position.z = 0.6

        if self.target_pose.pose.position.z > 3.0:
            self.target_pose.pose.position.z = 3.0

    def order_callback(self, msg):
        self.node.get_logger().info('Order: "%s"' % msg.data)
        if msg.data == 'take_off':
            if self._is_flying:
                self.node.get_logger().warning('Already flying')
            else:
                self.take_off()
        elif msg.data == 'land':
            if self._is_flying:
                self.descent()
                self.formation = False
            else:
                self.node.get_logger().warning('In land')
        elif not msg.data.find("rele_air"):
            if msg.data.find("_p_") == 8:
                if self.rele_p:
                    self.rele_p = False
                else:
                    self.rele_p = True
            elif msg.data.find("_s_") == 8:
                if self.rele_s:
                    self.rele_s = False
                else:
                    self.rele_s = True
            elif msg.data.find("_a_") == 8:
                if self.rele_a:
                    self.rele_a = False
                else:
                    self.rele_a = True
            elif msg.data.find("_r_") == 8:
                if self.rele_r:
                    self.rele_r = False
                else:
                    self.rele_r = True
            if msg.data.find("_x") == 10:
                if self.rele_x:
                    self.rele_x = False
                else:
                    self.rele_x = True
            if msg.data.find("_y") == 10:
                if self.rele_y:
                    self.rele_y = False
                else:
                    self.rele_y = True
            if msg.data.find("_z") == 10:
                if self.rele_z:
                    self.rele_z = False
                else:
                    self.rele_z = True

        elif msg.data == 'formation_run':
            if self.task_config:
                self.formation = True
        elif msg.data == 'formation_stop':
            self.formation = False
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
            if self.pose.position.z < 0.7:
                self.Fixed_z = True
            self.update_gain = True
            self.state = [10.0, 10.0, 10.0, 10.0, 10.0]
            for agent in self.agent_list:
                if agent.id == 'origin':
                    agent.k = 4.0
        elif not msg.data.find("remove") == -1 and self.task_config:
            self.remove_agent(msg.data)
        elif not msg.data.find("add") == -1 and self.task_config:
            self.add_agent(msg.data)
        else:
            self.node.get_logger().error('"%s": Unknown order' % (msg.data))

    def swarm_goalpose_callback(self, msg):
        if not self.centroid_leader:
            self.node.get_logger().info('Formation Control::Leader-> Centroid.')
        self.centroid_leader = True
        self.leader_cmd = msg

    def take_off(self):
        self.node.get_logger().info('Take Off...')
        self.target_pose.pose.position.z = 1.1
        self._is_flying = True
        self.t_ready = Timer(4, self._ready)
        self.t_ready.start()

    def _ready(self):
        self.node.get_logger().info('Ready!!.')
        self.ready = True
        if self.id == 'dron01':
            msg = String()
            msg.data = 'ready'
            self.swarm_status_publisher.publish(msg)

    def descent(self):
        self.target_pose.pose.position.z = 0.6
        self.node.get_logger().info('Descent...')
        self.t_desc = Timer(2, self.take_land)
        self._is_flying = False
        self.ready = False
        self.t_desc.start()

    def take_land(self):
        self.node.get_logger().info('Take Land.')
        self.target_pose.pose.position.z = 0.1
        self.t_desc = Timer(2, self.stop)
        self.init_pose = False

    def stop(self):
        self.target_pose.pose.position.z = 0.0

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
                agent.parent.node.destroy_publisher(agent.publisher_marker_)
                self.agent_list.pop(j)
            else:
                j += 1

    def _disconnected(self):
        self.node.get_logger().info('TO-DO Disconnect.')
        # self.descent()
        self.target_pose.pose.position.x = self.target_pose.pose.position.x * 1.2
        self.target_pose.pose.position.y = self.target_pose.pose.position.y * 1.2
        self.target_pose.pose.position.z = self.target_pose.pose.position.z * 0.8
        self.disconnect = True
        self.node.destroy_publisher(self.pose_publisher)
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
            agent.parent.node.destroy_publisher(agent.publisher_marker_)
            msg = String()
            msg.data = 'remove_' + self.id
            agent.publisher_order_.publish(msg)

    ###############
    #    Tasks    #
    ###############
    def load_formation_params(self):
        if "type" in self.config['task']:
            task_type = self.config['task']['type']
        else:
            task_type = 'distance'
        if "role" in self.config['task']:
            role = self.config['task']['role']
        else:
            role = 'consensus'
        if "controller" in self.config['task']:
            self.controller = self.config['task']['controller']
        if "type" in self.controller:
            self.controller_type = self.controller['type']
        else:
            self.controller_type = 'gradient_p'
        if "gain" in self.controller:
            self.k = self.controller['gain']
        else:
            self.k = 0.1
        if "upperLimit" in self.controller:
            self.ul = sqrt(pow(self.controller['upperLimit'], 2) +
                           pow(self.controller['upperLimit'], 2) +
                           pow(self.controller['upperLimit'], 2))
        else:
            self.ul = sqrt(pow(0.3, 2) + pow(0.3, 2) + pow(0.3, 2))
        if "lowerLimit" in self.controller:
            self.ll = -sqrt(pow(self.controller['lowerLimit'],
                                2) + pow(self.controller['lowerLimit'],
                                         2) + pow(self.controller['lowerLimit'],
                                                  2))
        else:
            self.ll = -sqrt(pow(0.3, 2) + pow(0.3, 2) + pow(0.3, 2))
        if "protocol" in self.controller:
            self.continuous = self.controller['protocol'] == 'Continuous'
        else:
            self.continuous = True
        if "threshold" in self.controller:
            self.trigger_ai = self.controller['threshold']['ai']
            self.trigger_co = self.controller['threshold']['co']
        if "period" in self.controller:
            self.task_period = self.controller['period']
        else:
            self.task_period = 0.02

        self.node.destroy_subscription(self.sub_goalpose)
        self.publisher_goalpose = self.node.create_publisher(
            PoseStamped, self.id + '/goal_pose', 10)
        self.publisher_global_error_ = self.node.create_publisher(
            Float64, self.id + '/global_error', 10)
        self.node.get_logger().debug('Task %s by %s' % (task_type, role))

        self.event_x = self.node.create_publisher(Bool, self.id + '/formation/event_x', 10)
        self.event_y = self.node.create_publisher(Bool, self.id + '/formation/event_y', 10)
        self.event_z = self.node.create_publisher(Bool, self.id + '/formation/event_z', 10)

        if self.controller_type == 'pid':
            self.formation_x_controller = PIDController(
                self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)
            self.formation_y_controller = PIDController(
                self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)
            self.formation_z_controller = PIDController(
                self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)

        self.agent_list = list()
        aux = self.config['task']['relationship']
        self.relationship = aux.split(', ')
        if task_type == 'distance':
            if any(t in self.controller_type for t in ('gradient', 'ML1', 'ML2', 'ML3')):
                self.node.create_timer(self.task_period, self.distance_gradient_controller)
            for rel in self.relationship:
                self.N = self.N + 1.0
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
                    robot = Agent(self, self.node, id, point=p, vector=u)
                    self.node.get_logger().debug(
                        'Agent: %s: Neighbour: %s ::: Px: %s Py: %s Pz: %s' %
                        (self.id, id, aux[1], aux[2], aux[3]))
                else:
                    if aux[0] == 'origin' or aux[0] == 'sphere':
                        auxa = aux[1]
                        auxb = auxa.split('-')
                        origin = Point()
                        origin.x = float(auxb[1])
                        origin.y = float(auxb[2])
                        origin.z = float(auxb[3])
                        self.R = float(auxb[0])
                        self.geometry = Sphere(self.R, origin)
                        robot = Agent(self, self.node, aux[0], d=float(auxb[0]), point=origin)
                    elif aux[0] == 'cone':
                        auxa = aux[1]
                        auxb = auxa.split('-')
                        origin = Point()
                        origin.x = float(auxb[2])
                        origin.y = float(auxb[3])
                        origin.z = float(auxb[4])
                        self.geometry = Cone(float(auxb[0]), float(auxb[1]), origin)
                        self.node.get_logger().info(
                            'Agent: %s: Cono: a %s \tc: %s' %
                            (self.id, auxb[0], auxb[1]))
                        robot = Agent(self, self.node, aux[0], d=float(auxb[0]), point=origin)
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
                        self.geometry = Ellipsoid(a=a, b=b, c=c, origin=origin)
                        robot = Agent(self, self.node, aux[0], a=a, b=b, c=c, point=origin, d=0.0)
                    else:
                        robot = Agent(self, self.node, aux[0], d=float(aux[1]))
                    self.node.get_logger().debug(
                        'Agent: %s: Neighbour: %s \td: %s' %
                        (self.id, aux[0], aux[1]))
                self.agent_list.append(robot)
        elif task_type == 'pose':
            if any(t in self.controller_type for t in ('gradient', 'ML1', 'ML2', 'ML3')):
                self.node.create_timer(self.controller['period'], self.pose_gradient_controller)
            for rel in self.relationship:
                aux = rel.split('_')
                robot = Agent(
                    self, self.node, aux[0], x=float(
                        aux[1]), y=float(
                        aux[2]), z=float(
                        aux[3]))
                self.node.get_logger().debug(
                    'Agent: %s. Neighbour %s :::x: %s \ty: %s \tz: %s' %
                    (self.id, aux[0], aux[1], aux[2], aux[3]))
                self.agent_list.append(robot)

    def distance_gradient_controller(self):
        if self.formation and not self.disconnect:
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

                if not self.digital_twin:
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
            msg.data = [round(dx, 3), round(dy, 3), round(dz, 3), self.N]
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 4
            msg.layout.dim[0].stride = 1
            self.publisher_mrs_data.publish(msg)
            msg_data = Float64()
            msg_data.data = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
            self.publisher_mrs_data_mod.publish(msg_data)

            if abs(msg_data.data) > self.ul:
                dx = (dx / msg_data.data) * self.ul
                dy = (dy / msg_data.data) * self.ul
                dz = (dz / msg_data.data) * self.ul

            if self.pose.position.z < 0.6 and dz < 0.0 and '_v' in self.controller_type:
                dz = 0.0

            if self.pose.position.z > 3.0 and dz > 0.0 and '_v' in self.controller_type:
                dz = 0.0

            msg.data = [round(dx, 3), round(dy, 3), round(dz, 3), self.N]
            self.publisher_mrs_data_gt.publish(msg)
            msg_data = Float64()
            msg_data.data = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dx, 2))
            self.publisher_mrs_data_gt_mod.publish(msg_data)

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

            if self.Fixed_z:
                self.target_pose.pose.position.z = 0.6

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

        dt = self.robot.getTime() - self.past_time

        if not self.init_pose:
            self.past_x_global = self.gps.getValues()[0]
            self.target_pose.pose.position.x = self.gps.getValues()[0]
            self.past_y_global = self.gps.getValues()[1]
            self.target_pose.pose.position.y = self.gps.getValues()[1]
            self.init_pose = True
            self.last_pose.position.x = self.gps.getValues()[0]
            self.last_pose.position.y = self.gps.getValues()[1]
            self.last_pose.position.z = self.gps.getValues()[2]
            roll = self.imu.getRollPitchYaw()[0]
            pitch = self.imu.getRollPitchYaw()[1]
            yaw = self.imu.getRollPitchYaw()[2]
            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.last_pose.orientation.x = q[0]
            self.last_pose.orientation.y = q[1]
            self.last_pose.orientation.z = q[2]
            self.last_pose.orientation.w = q[3]
            init_pose = PoseStamped()
            init_pose.header.frame_id = "map"
            init_pose.pose = self.last_pose
            init_pose.header.stamp = self.node.get_clock().now().to_msg()
            if not self.disconnect:
                self.pose_publisher.publish(init_pose)
            self.z_controller.past_time = self.past_time
            self.w_controller.past_time = self.past_time
            # self.take_off()

        # Get measurements
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        self.global_yaw = yaw
        roll_rate = self.gyro.getValues()[0]
        pitch_rate = self.gyro.getValues()[1]
        yaw_rate = self.gyro.getValues()[2]
        x_global = self.gps.getValues()[0]  # - self.first_x_global
        vx_global = (x_global - self.past_x_global) / dt
        y_global = self.gps.getValues()[1]  # - self.first_y_global
        vy_global = (y_global - self.past_y_global) / dt
        z_global = self.gps.getValues()[2]
        vz_global = (z_global - self.past_z_global) / dt

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        self.pose.position.x = x_global
        self.pose.position.y = y_global
        self.pose.position.z = z_global
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        t_base = TransformStamped()
        t_base.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        t_base.header.frame_id = 'map'
        if self.digital_twin:
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

        delta = np.array([self.pose.position.x -
                          self.last_pose.position.x, self.pose.position.y -
                          self.last_pose.position.y, self.pose.position.z -
                          self.last_pose.position.z])

        if (self.communication or np.linalg.norm(delta) > 0.01) and not self.disconnect:
            PoseStamp = PoseStamped()
            PoseStamp.pose.position.x = self.pose.position.x
            PoseStamp.pose.position.y = self.pose.position.y
            PoseStamp.pose.position.z = self.pose.position.z
            PoseStamp.pose.orientation.x = self.pose.orientation.x
            PoseStamp.pose.orientation.y = self.pose.orientation.y
            PoseStamp.pose.orientation.z = self.pose.orientation.z
            PoseStamp.pose.orientation.w = self.pose.orientation.w
            PoseStamp.header.frame_id = "map"
            PoseStamp.header.stamp = self.node.get_clock().now().to_msg()
            if self.path_enable:
                self.path.header.stamp = self.node.get_clock().now().to_msg()
                self.path.poses.append(PoseStamp)
                self.path_publisher.publish(self.path)
            self.pose_publisher_gt.publish(PoseStamp)
            if 'ML2' in self.controller_type:
                PoseStamp.pose.position = self.geometry.projection(self.pose.position)
                self.pose_publisher.publish(PoseStamp)
            else:
                self.pose_publisher.publish(PoseStamp)

            self.last_pose.position.x = self.pose.position.x
            self.last_pose.position.y = self.pose.position.y
            self.last_pose.position.z = self.pose.position.z

        # Position Controller
        # Z Controller
        if not self.formation or '_p' in self.controller_type:
            if True or not (self.rele_p and self.rele_z):
                if self.z_controller.eval_threshold(
                        z_global, self.target_pose.pose.position.z) or self.continuous:
                    self.z_controller.error[0] = (self.target_pose.pose.position.z - z_global)
                    dtz = self.robot.getTime() - self.z_controller.past_time
                    self.target_twist.linear.z = self.z_controller.update(dtz)
                    self.z_controller.past_time = self.robot.getTime()
                    if not self.communication:
                        msg = Bool()
                        msg.data = True
                        self.event_z_.publish(msg)
                    self.node.get_logger().debug(
                        'Z Controller Event. Th: %.4f; Inc: %.4f; dT: %.4f' %
                        (self.z_controller.th, self.z_controller.inc, dtz))
                else:
                    self.target_twist.linear.z = self.z_controller.last_value
                if False and self.rele_z:
                    self.rele_cmd_eq = vz_global
                self.node.get_logger().debug(
                    'Z: R: %.2f P: %.2f C: %.2f' %
                    (self.target_pose.pose.position.z, z_global, self.target_twist.linear.z))
            else:
                error = self.target_pose.pose.position.z - z_global
                self.target_twist.linear.z = self.update_position_relay(error)

        if True or not (self.rele_s and self.rele_z):
            if self.w_controller.eval_threshold(
                    vz_global, self.target_twist.linear.z) or self.continuous:
                self.w_controller.error[0] = (self.target_twist.linear.z - vz_global)
                dtw = self.robot.getTime() - self.w_controller.past_time
                cmd_thrust = self.w_controller.update(dtw) * 1000 + 38000
                self.w_controller.past_time = self.robot.getTime()
                self.node.get_logger(
                    ).debug('W Controller Event.V: %.2f; dT: %.3f' % (cmd_thrust, dtw))
            else:
                cmd_thrust = self.w_controller.last_value * 1000 + 38000
            if self.rele_z and False:
                self.rele_cmd_eq = cmd_thrust
            self.node.get_logger().debug(
                'dZ: R: %.2f P: %.2f C: %.2f' %
                (self.target_twist.linear.z, vz_global, cmd_thrust))
        else:
            self.z_controller.integral = 0.0
            error = self.target_twist.linear.z - vz_global
            cmd_thrust = self.update_speed_relay(error)

        # X-Y Controller
        # IPC Controller
        if self.controller_IPC:
            self.target_twist = self.IPC_controller()
            # Save zone
            delta = np.array([self.pose.position.x -
                              self.target_pose.pose.position.x, self.pose.position.y -
                              self.target_pose.pose.position.y])
            if np.linalg.norm(delta) < 0.02:
                self.target_twist.angular.z = 0.0
                self.node.get_logger().info('Save zone')
                # X Controller
                if self.x_controller.eval_threshold(
                        x_global, self.target_pose.pose.position.x) or self.continuous:
                    self.x_controller.error[0] = self.target_pose.pose.position.x - x_global
                    dtx = self.robot.getTime() - self.x_controller.past_time
                    self.target_twist.linear.x = self.x_controller.update(dtx)
                    self.x_controller.past_time = self.robot.getTime()
                    if not self.communication:
                        msg = Bool()
                        msg.data = True
                        self.event_x_.publish(msg)
                else:
                    self.target_twist.linear.x = self.x_controller.last_value
                self.node.get_logger().debug(
                    'X: R: %.2f P: %.2f C: %.2f' %
                    (self.target_pose.pose.position.x, x_global, self.target_twist.linear.x))
                # Y Controller
                if self.y_controller.eval_threshold(
                        y_global, self.target_pose.pose.position.y) or self.continuous:
                    self.y_controller.error[0] = self.target_pose.pose.position.y - y_global
                    dty = self.robot.getTime() - self.y_controller.past_time
                    self.target_twist.linear.y = self.y_controller.update(dty)
                    self.y_controller.past_time = self.robot.getTime()
                    if not self.communication:
                        msg = Bool()
                        msg.data = True
                        self.event_y_.publish(msg)
                else:
                    self.target_twist.linear.y = self.y_controller.last_value
                self.node.get_logger().debug(
                    'Y: R: %.2f P: %.2f C: %.2f' %
                    (self.target_pose.pose.position.y, y_global, self.target_twist.linear.y))

        if self.controller_PID and (not self.formation or '_p' in self.controller_type):
            # X Controller
            if True or not (self.rele_p and self.rele_x):
                if self.x_controller.eval_threshold(
                        x_global, self.target_pose.pose.position.x) or self.continuous:
                    self.x_controller.error[0] = self.target_pose.pose.position.x - x_global
                    dtx = self.robot.getTime() - self.x_controller.past_time
                    self.target_twist.linear.x = self.x_controller.update(dtx)
                    self.x_controller.past_time = self.robot.getTime()
                    if not self.communication:
                        msg = Bool()
                        msg.data = True
                        self.event_x_.publish(msg)
                else:
                    self.target_twist.linear.x = self.x_controller.last_value
                if self.rele_x:
                    self.rele_cmd_eq = vx_global
                self.node.get_logger().debug(
                    'X: R: %.2f P: %.2f C: %.2f' %
                    (self.target_pose.pose.position.x, x_global, self.target_twist.linear.x))
            else:
                error = self.target_pose.pose.position.x - x_global
                self.node.get_logger().info('Error: %.2f' % error)
                self.target_twist.linear.x = self.update_position_relay(error)
                self.node.get_logger().info('CMD: %.2f' % self.target_twist.linear.x)
            # Y Controller
            if True or not (self.rele_p and self.rele_y):
                if self.y_controller.eval_threshold(
                        y_global, self.target_pose.pose.position.y) or self.continuous:
                    self.y_controller.error[0] = self.target_pose.pose.position.y - y_global
                    dty = self.robot.getTime() - self.y_controller.past_time
                    self.target_twist.linear.y = self.y_controller.update(dty)
                    self.y_controller.past_time = self.robot.getTime()
                    if not self.communication:
                        msg = Bool()
                        msg.data = True
                        self.event_y_.publish(msg)
                else:
                    self.target_twist.linear.y = self.y_controller.last_value
                if self.rele_y:
                    self.rele_cmd_eq = vy_global
                self.node.get_logger().debug(
                    'Y: R: %.2f P: %.2f C: %.2f' %
                    (self.target_pose.pose.position.y, y_global, self.target_twist.linear.y))
            else:
                error = self.target_pose.pose.position.y - y_global
                self.target_twist.linear.y = self.update_position_relay(error)

        # dX-dY Controller
        if True or not (self.rele_s and self.rele_x):
            if self.u_controller.eval_threshold(
                    vx_global, self.target_twist.linear.x) or self.continuous:
                self.u_controller.error[0] = (
                    self.target_twist.linear.x - vx_global) * cos(yaw) + (
                    self.target_twist.linear.y - vy_global) * sin(yaw)
                dtu = self.robot.getTime() - self.u_controller.past_time
                pitch_ref = self.u_controller.update(dtu)
                self.u_controller.past_time = self.robot.getTime()
            else:
                pitch_ref = self.u_controller.last_value
            if self.rele_x and False:
                self.rele_cmd_eq = pitch_ref
        else:
            error = (self.target_twist.linear.x - vx_global) * cos(yaw) + \
                (self.target_twist.linear.y - vy_global) * sin(yaw)
            pitch_ref = self.update_speed_relay(error)

        if True or not (self.rele_s and self.rele_y):
            self.v_controller.error[0] = -(self.target_twist.linear.x - vx_global) * \
                sin(yaw) + (self.target_twist.linear.y - vy_global) * cos(yaw)
            dtv = self.robot.getTime() - self.v_controller.past_time
            roll_ref = self.v_controller.update(dtv)
            self.v_controller.past_time = self.robot.getTime()
            if self.rele_y and False:
                self.rele_cmd_eq = roll_ref
        else:
            error = -(-(self.target_twist.linear.x - vx_global) * sin(yaw) +
                      (self.target_twist.linear.y - vy_global) * cos(yaw))
            self.node.get_logger().debug('Error: %.2f' % error)
            roll_ref = self.update_speed_relay(error)

        if self.publisher_twist_enable:
            msg = Twist()
            msg = self.target_twist
            self.publisher_twist.publish(msg)
        # Attitude Controller
        # Pitch Controller
        self.pitch_controller.error[0] = pitch_ref - degrees(pitch)
        if not (self.rele_a and self.rele_y):
            dpitch_ref = self.pitch_controller.update(dt)
            if self.rele_y and False:
                self.rele_cmd_eq = dpitch_ref
        else:
            self.pitch_controller.integral = 0.0
            dpitch_ref = self.update_attitude_relay(self.pitch_controller.error[0])
        # Roll Controller
        self.roll_controller.error[0] = roll_ref - degrees(roll)
        if not (self.rele_a and self.rele_x):
            droll_ref = self.roll_controller.update(dt)
            if self.rele_x and False:
                self.rele_cmd_eq = droll_ref
        else:
            self.roll_controller.integral = 0.0
            droll_ref = self.update_attitude_relay(self.roll_controller.error[0])
        # Yaw Controller
        angles = tf_transformations.euler_from_quaternion(
            (self.target_pose.pose.orientation.x,
             self.target_pose.pose.orientation.y,
             self.target_pose.pose.orientation.z,
             self.target_pose.pose.orientation.w))
        self.yaw_controller.error[0] = angles[2] - yaw
        if self.yaw_controller.error[0] > pi:
            self.yaw_controller.error[0] = self.yaw_controller.error[0] - 2 * pi
        if self.yaw_controller.error[0] < -pi:
            self.yaw_controller.error[0] = self.yaw_controller.error[0] + 2 * pi
        if not (self.rele_a and self.rele_z):
            dyaw_ref = self.yaw_controller.update(dt)
            if self.rele_z:
                self.rele_cmd_eq = dyaw_ref
        else:
            self.yaw_controller.integral = 0.0
            droll_ref = self.update_attitude_relay(self.yaw_controller.error[0])

        if self.publisher_data_attitude_enable:
            msg = Float64MultiArray()
            msg.data = {roll_ref, pitch_ref, angles[2], degrees(roll), degrees(pitch), yaw}
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 6
            msg.layout.dim[0].stride = 1
            self.publisher_data_attitude.publish(msg)

        # Rate Controller
        self.dpitch_controller.error[0] = dpitch_ref - degrees(pitch_rate)
        delta_pitch = self.dpitch_controller.update(dt)
        self.droll_controller.error[0] = droll_ref - degrees(roll_rate)
        delta_roll = self.droll_controller.update(dt)
        self.dyaw_controller.error[0] = dyaw_ref - degrees(yaw_rate)
        delta_yaw = self.dyaw_controller.update(dt)

        if self.publisher_data_rate_enable:
            msg = Float64MultiArray()
            msg.data = {
                droll_ref,
                dpitch_ref,
                dyaw_ref,
                degrees(roll_rate),
                degrees(pitch_rate),
                degrees(yaw_rate)}
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 6
            msg.layout.dim[0].stride = 1
            self.publisher_data_rate.publish(msg)

        self.node.get_logger().debug(
            'IPC:: V: %.4f W: %.4f' %
            (self.target_twist.linear.x, dyaw_ref))
        if self.publisher_data_enable:
            msg = Float64MultiArray()
            msg.data = {delta_roll, delta_pitch, delta_yaw}
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 3
            msg.layout.dim[0].stride = 1
            self.publisher_data.publish(msg)

        # Motor mixing Controller
        motorPower_m1 = (cmd_thrust - 0.5 * delta_roll - 0.5 * delta_pitch + delta_yaw)
        motorPower_m2 = (cmd_thrust - 0.5 * delta_roll + 0.5 * delta_pitch - delta_yaw)
        motorPower_m3 = (cmd_thrust + 0.5 * delta_roll + 0.5 * delta_pitch + delta_yaw)
        motorPower_m4 = (cmd_thrust + 0.5 * delta_roll - 0.5 * delta_pitch - delta_yaw)

        if self.publisher_data_motor_enable:
            msg = Float64MultiArray()
            msg.data = {cmd_thrust, motorPower_m1, motorPower_m2, motorPower_m3, motorPower_m4}
            msg.layout.data_offset = 0
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = 'data'
            msg.layout.dim[0].size = 5
            msg.layout.dim[0].stride = 1
            self.publisher_data_motor.publish(msg)

        self.node.get_logger().debug(
            'Thrust(C): %.2f CRoll(C): %.2f CPitch(C): %.2f CYaw(C): %.2f' %
            (cmd_thrust, delta_roll, delta_pitch, delta_yaw))

        scaling = 1000  # TO-DO, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(-motorPower_m1 / scaling)
        self.m2_motor.setVelocity(motorPower_m2 / scaling)
        self.m3_motor.setVelocity(-motorPower_m3 / scaling)
        self.m4_motor.setVelocity(motorPower_m4 / scaling)

        self.past_time = self.robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global
        self.past_z_global = z_global

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
