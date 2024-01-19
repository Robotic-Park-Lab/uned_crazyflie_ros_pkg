import rclpy
import os
from rclpy.node import Node
from rclpy.time import Time
from threading import Timer
import yaml

from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker

from math import atan2, cos, sin, degrees, radians, pi, sqrt
import numpy as np
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Change this path to your crazyflie-firmware folder
# sys.path.append('/home/kiko/Code/crazyflie-firmware')
# import cffirmware

class Agent():
    def __init__(self, parent, id, x = None, y = None, z = None, d = None, point = None, vector = None):
        self.id = id
        self.distance = False
        self.parent = parent
        self.pose = Pose()
        if not id.find("line") == -1:
            self.distance_bool = True
            self.d = 0
            self.point = point
            self.vector = vector
            self.mod=pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2)
            self.k = 4.0
        else:
            if d == None:
                self.distance_bool = False
                self.x = x
                self.y = y
                self.z = z
                self.parent.node.get_logger().info('Agent: %s' % self.str_())
            else:
                self.distance_bool = True
                self.d = d
                self.parent.node.get_logger().info('Agent: %s' % self.str_distance_())
            
            if self.id == 'origin':
                self.pose.position.x = 0.0
                self.pose.position.y = 0.0
                self.pose.position.z = 0.8
                self.k = 2.0
                self.sub_pose = self.parent.node.create_subscription(PoseStamped, self.id + '/local_pose', self.gtpose_callback, 10)
            else:
                self.k = 1.0
                self.sub_pose = self.parent.node.create_subscription(PoseStamped, self.id + '/local_pose', self.gtpose_callback, 10)
        if not self.parent.digital_twin:
            self.sub_d_ = self.parent.node.create_subscription(Float64, '/' + self.id + '/d', self.d_callback, 10)
            self.publisher_data_ = self.parent.node.create_publisher(Float64, self.parent.name_value + '/' + self.id + '/data', 10)
            self.publisher_marker_ = self.parent.node.create_publisher(Marker, self.parent.name_value + '/' + self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))
    
    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def d_callback(self, msg):
        self.d = msg.data

    def gtpose_callback(self, msg):
        self.pose = msg.pose
        if not self.parent.digital_twin and self.parent.config['task']['enable']:
            line = Marker()
            p0 = Point()
            p0.x = self.parent.gt_pose.position.x
            p0.y = self.parent.gt_pose.position.y
            p0.z = self.parent.gt_pose.position.z

            p1 = Point()
            p1.x = self.pose.position.x
            p1.y = self.pose.position.y
            p1.z = self.pose.position.z

            line.header.frame_id = 'map'
            line.header.stamp = self.parent.node.get_clock().now().to_msg()
            line.id = 1
            line.type = 5
            line.action = 0
            line.scale.x = 0.01
            line.scale.y = 0.01
            line.scale.z = 0.01
            
            if self.distance_bool:
                distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
                e = abs(distance - self.d)
            else:
                e = sqrt(pow(self.x-(p0.x-p1.x),2)+pow(self.y-(p0.y-p1.y),2)+pow(self.z-(p0.z-p1.z),2))

            if e > 0.05:
                line.color.r = 1.0
            else:
                if e > 0.025:
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


class CrazyflieWebotsDriver:
    def init(self, webots_node, properties):
        
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())

        ## Initialize motors
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

        ## Initialize Sensors
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

        ## Intialize Variables
        self.target_twist = Twist()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.z = 0.0
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_z_global = 0
        self.past_time = self.robot.getTime()
        self._is_flying = False
        self.distance_formation_bool_update = True
        self.gt_pose = Pose()

        self.first_pos = True
        self.first_x_global = 0.0
        self.first_y_global = 0.0

        self.centroid_leader = False
        self.leader_cmd = PoseStamped()
        self.leader_cmd.header.frame_id = "map"
        self.last_pose = Pose()

        ## Read config_file
        self.config_file = properties.get("config_file")
        with open(self.config_file, 'r') as file:
            documents = yaml.safe_load(file)
        for robot in documents['Robots']:
            if documents['Robots'][robot]['name'] == properties.get("name_id"):
                self.config = documents['Robots'][robot]

        ## Intialize Controllers
        self.continuous = True
        # IPC
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
        # Position
        k = 0.5
        self.z_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.1, 0.01)
        self.x_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, k, -k, 0.1, 0.01)
        self.y_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, k, -k, 0.1, 0.01)
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

        # cffirmware.controllerPidInit()

        # Init ROS2 Node
        self.name_value = self.config['name']
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.name_value+'_driver')
        self.digital_twin = self.config['type'] == 'digital_twin'
        # Subscription
        self.node.create_subscription(Twist, self.name_value+'/cmd_vel', self.cmd_vel_callback, 1)
        self.node.create_subscription(PoseStamped, self.name_value+'/pose_dt', self.dt_pose_callback, 1)
        self.sub_goalpose = self.node.create_subscription(PoseStamped, self.name_value+'/goal_pose', self.goal_pose_callback, 1)
        self.node.create_subscription(String, self.name_value+'/order', self.order_callback, 1)
        self.node.create_subscription(String, 'swarm/order', self.order_callback, 1)
        self.node.create_subscription(PoseStamped, 'swarm/goal_pose', self.swarm_goalpose_callback, 1)
        # Publisher
        self.laser_publisher = self.node.create_publisher(LaserScan, self.name_value+'/scan', 10)
        self.event_x_ = self.node.create_publisher(Bool, self.name_value+'/event_x', 10)
        self.event_y_ = self.node.create_publisher(Bool, self.name_value+'/event_y', 10)
        self.event_z_ = self.node.create_publisher(Bool, self.name_value+'/event_z', 10)
        self.path_publisher = self.node.create_publisher(Path, self.name_value+'/path', 10)
        self.swarm_status_publisher = self.node.create_publisher(String, 'swarm/status', 10)

        self.odom_publisher = self.node.create_publisher(Odometry, self.name_value+'/odom', 10)
        if self.digital_twin:
            pose_name = self.name_value+'/dt_pose'
        else:
            pose_name = self.name_value+'/local_pose'
        self.pose_publisher = self.node.create_publisher(PoseStamped, pose_name, 10)

        self.tfbr = TransformBroadcaster(self.node)
        self.msg_laser = LaserScan()
        self.node.create_timer(0.2, self.publish_laserscan_data)
        
        self.path = Path()
        self.path.header.frame_id = "map"

        self.continuous = False
        self.trigger_ai = 0.01
        self.trigger_co = 0.1
        self.trigger_last_signal = 0.0
        self.formation_bool = False

        self.initialize()

    def initialize(self):
        self.node.get_logger().info('Webots_Node::inicialize() ok. %s' % (str(self.name_value)))
        # Read Params
        
        # Set Formation
        if self.config['task']['enable']:
            self.node.destroy_subscription(self.sub_goalpose)
            self.publisher_goalpose = self.node.create_publisher(PoseStamped, self.name_value + '/goal_pose', 10)
        if self.config['task']['enable'] or self.config['task']['Onboard']:
            self.node.get_logger().info('Task %s by %s' % (self.config['task']['type'], self.config['task']['role']))
            self.controller = self.config['task']['controller']
            self.controller_type = self.controller['type']
            self.k = self.controller['gain']
            self.ul = self.controller['upperLimit']
            self.ll = self.controller['lowerLimit']
            self.continuous = self.controller['protocol'] == 'Continuous'
            self.event_x = self.node.create_publisher(Bool, self.name_value + '/event_x', 10)
            self.event_y = self.node.create_publisher(Bool, self.name_value + '/event_y', 10)
            self.event_z = self.node.create_publisher(Bool, self.name_value + '/event_z', 10)

            if not self.continuous:
                self.trigger_ai = self.controller['threshold']['ai']
                self.trigger_co = self.controller['threshold']['co']

            if self.controller_type == 'pid':
                self.formation_x_controller = PIDController(self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)
                self.formation_y_controller = PIDController(self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)
                self.formation_z_controller = PIDController(self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)

            self.agent_list = list()
            aux = self.config['task']['relationship']
            self.relationship = aux.split(', ')
            if self.config['task']['type'] == 'distance':
                if self.controller_type == 'gradient':
                    self.node.create_timer(self.controller['period'], self.distance_gradient_controller)
                elif self.controller_type == 'pid':
                    self.node.create_timer(self.controller['period'], self.distance_pid_controller)
                for rel in self.relationship:
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
                        robot = Agent(self, id, point = p, vector = u)
                        self.node.get_logger().info('Agent: %s: Neighbour: %s ::: Px: %s Py: %s Pz: %s' % (self.name_value, id, aux[1], aux[2], aux[3]))
                    else:
                        robot = Agent(self, aux[0], d = float(aux[1]))
                        self.node.get_logger().info('Agent: %s: Neighbour: %s \td: %s' % (self.name_value, aux[0], aux[1]))
                    self.agent_list.append(robot)
            elif self.config['task']['type'] == 'pose':
                if self.controller_type == 'gradient':
                    self.node.create_timer(self.controller['period'], self.pose_gradient_controller)
                elif self.controller_type == 'pid':
                    self.node.create_timer(self.controller['period'], self.pose_pid_controller)
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, aux[0], x = float(aux[1]), y = float(aux[2]), z = float(aux[3]))
                    self.node.get_logger().info('Agent: %s. Neighbour %s :::x: %s \ty: %s \tz: %s' % (self.name_value, aux[0], aux[1], aux[2], aux[3]))
                    self.agent_list.append(robot)

        self.path_enable = self.config['local_pose']['path']
        self.communication = (self.config['communication']['type'] == 'Continuous')
        if not self.communication:
            self.threshold = self.config['communication']['threshold']['co']
        else:
            self.threshold = 0.001

    def publish_laserscan_data(self):
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
        self.msg_laser.header.frame_id = self.name_value
        self.msg_laser.range_min = 0.1
        self.msg_laser.range_max = max_range
        self.msg_laser.ranges = [back_range, left_range, front_range, right_range, back_range]
        self.msg_laser.angle_min = 0.5 * 2*pi
        self.msg_laser.angle_max =  -0.5 * 2*pi
        self.msg_laser.angle_increment = -1.0*pi/2
        self.laser_publisher.publish(self.msg_laser)

    def dt_pose_callback(self, pose):
        self.node.get_logger().info('TO-DO: DT Pose: X:%f Y:%f' % (pose.pose.position.x,pose.pose.position.y))
        # self.robot.getSelf().getField("translation").setSFVec3f([pose.position.x, pose.position.y, pose.position.z])
        # self.robot.getSelf().getField("rotation").setSFVec3f([0.0, 0.0, 0.0])

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def goal_pose_callback(self, pose):
        self.target_pose = pose

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
                self.formation_bool = False
            else:
                self.node.get_logger().warning('In land')
        elif msg.data == 'formation_run':
            if self.config['task']['enable']:
                self.formation_bool = True
        elif msg.data == 'formation_stop':
            self.formation_bool = False
        elif not msg.data.find("agent") == -1 and self.config['task']['enable']:
            aux = msg.data.split('_')
            if aux[1] == 'remove':
                print(self.agent_list)
        else:
            self.node.get_logger().error('"%s": Unknown order' % (msg.data))
    
    def swarm_goalpose_callback(self, msg):
        if not self.centroid_leader:
            self.node.get_logger().info('Formation Control::Leader-> Centroid.')
        self.centroid_leader = True
        self.leader_cmd = msg
        
    def take_off(self):
        self.node.get_logger().info('Take Off...')
        self.target_pose.pose.position.z = 1.0
        self._is_flying = True
        self.t_ready = Timer(4, self._ready)
        self.t_ready.start()

    def _ready(self):
        self.node.get_logger().info('Ready!!.')
        self.ready = True
        if self.name_value == 'dron01':
            msg = String()
            msg.data = 'ready'
            self.swarm_status_publisher.publish(msg)

    def descent(self):
        self.target_pose.pose.position.z = 0.8
        self.node.get_logger().info('Descent...')
        self.t_desc = Timer(2, self.take_land)
        self._is_flying = False
        self.ready = False
        self.t_desc.start()

    def take_land(self):
        self.node.get_logger().info('Take Land.')
        self.target_pose.pose.position.z = 0.6
        self.t_desc = Timer(2, self.stop)
        self.first_pos = False

    def stop(self):
        self.target_pose.pose.position.z = 0.0

    def distance_gradient_controller(self):
        if self.formation_bool:
            dx = dy = dz = 0
            for agent in self.agent_list:
                if not agent.id.find("line") == -1:
                    nearest = PoseStamped()
                    nearest.header.frame_id = "map"
                    gamma = -np.dot([agent.point.x-self.gt_pose.position.x, agent.point.y-self.gt_pose.position.y, agent.point.z-self.gt_pose.position.z],[agent.vector.x, agent.vector.y, agent.vector.z])/agent.mod
                    nearest.pose.position.x = agent.point.x + gamma * agent.vector.x
                    nearest.pose.position.y = agent.point.y + gamma * agent.vector.y
                    nearest.pose.position.z = agent.point.z + gamma * agent.vector.z
                    agent.gtpose_callback(nearest)
                error_x = self.gt_pose.position.x - agent.pose.position.x
                error_y = self.gt_pose.position.y - agent.pose.position.y
                error_z = self.gt_pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                d = sqrt(distance)
                if d == 0:
                    d = 1.0
                dx += self.k * agent.k * (pow(agent.d,2) - distance) * error_x/d
                dy += self.k * agent.k * (pow(agent.d,2) - distance) * error_y/d
                dz += self.k * agent.k * (pow(agent.d,2) - distance) * error_z/d
                
                if not self.digital_twin:
                    msg_data = Float64()
                    msg_data.data = abs(agent.d - d)
                    agent.publisher_data_.publish(msg_data)
                    self.node.get_logger().debug('Agent %s: D: %.2f dx: %.2f dy: %.2f dz: %.2f ' % (agent.id, msg_data.data, dx, dy, dz)) 
            
            if not self.continuous:
                delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
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

            self.target_pose.pose.position.x = self.gt_pose.position.x + dx
            self.target_pose.pose.position.y = self.gt_pose.position.y + dy
            self.target_pose.pose.position.z = self.gt_pose.position.z + dz

            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = tf_transformations.euler_from_quaternion((self.gt_pose.orientation.x, self.gt_pose.orientation.y, self.gt_pose.orientation.z, self.gt_pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = tf_transformations.quaternion_from_euler(roll, pitch, 0.0)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

            self.node.get_logger().debug('Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (self.gt_pose.position.x, self.target_pose.pose.position.x, self.gt_pose.position.y, self.target_pose.pose.position.y, self.gt_pose.position.z, self.target_pose.pose.position.z)) 
            if self.target_pose.pose.position.z < 0.4:
                self.target_pose.pose.position.z = 0.4

            if self.target_pose.pose.position.z > 2.0:
                self.target_pose.pose.position.z = 2.0

    def distance_pid_controller(self):
        if self.formation_bool:
            ex = ey = ez = 0
            for agent in self.agent_list:
                if not agent.id.find("line") == -1:
                    nearest = PoseStamped()
                    nearest.header.frame_id = "map"
                    gamma = -np.dot([agent.point.x-self.gt_pose.position.x, agent.point.y-self.gt_pose.position.y, agent.point.z-self.gt_pose.position.z],[agent.vector.x, agent.vector.y, agent.vector.z])/agent.mod
                    nearest.pose.position.x = agent.point.x + gamma * agent.vector.x
                    nearest.pose.position.y = agent.point.y + gamma * agent.vector.y
                    nearest.pose.position.z = agent.point.z + gamma * agent.vector.z
                    agent.gtpose_callback(nearest)
                error_x = self.gt_pose.position.x - agent.pose.position.x
                error_y = self.gt_pose.position.y - agent.pose.position.y
                error_z = self.gt_pose.position.z - agent.pose.position.z
                distance = sqrt(pow(error_x,2)+pow(error_y,2)+pow(error_z,2))
                e = agent.d - distance
                if distance == 0:
                    distance = 1.0
                ex += agent.k * e * (error_x/distance)
                ey += agent.k * e * (error_y/distance)
                ez += agent.k * e * (error_z/distance)
                
                if not self.digital_twin:
                    msg_data = Float64()
                    msg_data.data = e
                    agent.publisher_data_.publish(msg_data)

            aux = self.node.get_clock().now().to_msg()
            time = aux.sec + aux.nanosec*1e-9
                
            if not (self.formation_x_controller.eval_threshold(0.0, ex) or self.formation_y_controller.eval_threshold(0.0, ey) or self.formation_z_controller.eval_threshold(0.0, ez) or self.continuous):
                return
            
            msg = Bool()
            msg.data = True
            self.event_x.publish(msg)
            # X Controller
            self.formation_x_controller.error[0] = ex
            dtx = time - self.formation_x_controller.past_time
            dx = self.formation_x_controller.update(dtx)
            self.formation_x_controller.past_time = time

            # Y Controller
            self.formation_y_controller.error[0] = ey
            dty = time - self.formation_y_controller.past_time
            dy = self.formation_y_controller.update(dty)
            self.formation_y_controller.past_time = time

            # Z Controller
            self.formation_z_controller.error[0] = ez
            dtz = time - self.formation_z_controller.past_time
            dz = self.formation_z_controller.update(dtz)
            self.formation_z_controller.past_time = time

            self.target_pose.pose.position.x = self.gt_pose.position.x + dx
            self.target_pose.pose.position.y = self.gt_pose.position.y + dy
            self.target_pose.pose.position.z = self.gt_pose.position.z + dz

            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = tf_transformations.euler_from_quaternion((self.gt_pose.orientation.x, self.gt_pose.orientation.y, self.gt_pose.orientation.z, self.gt_pose.orientation.w))
                    
            if delta<0.1:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

            self.node.get_logger().debug('Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (self.gt_pose.position.x, self.target_pose.pose.position.x, self.gt_pose.position.y, self.target_pose.pose.position.y, self.gt_pose.position.z, self.target_pose.pose.position.z)) 
            if self.target_pose.pose.position.z < 0.4:
                self.target_pose.pose.position.z = 0.4

            if self.target_pose.pose.position.z > 2.0:
                self.target_pose.pose.position.z = 2.0
    
    def pose_gradient_controller(self):
        if self.formation_bool:
            dx = dy = dz = 0
            for agent in self.agent_list:
                error_x = self.gt_pose.position.x - agent.pose.position.x
                error_y = self.gt_pose.position.y - agent.pose.position.y
                error_z = self.gt_pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                d = sqrt(distance)
                dx += self.k * agent.k * (pow(agent.x,2) - pow(error_x,2)) * error_x/d
                dy += self.k * agent.k * (pow(agent.y,2) - pow(error_y,2)) * error_y/d
                dz += self.k * agent.k * (pow(agent.z,2) - pow(error_z,2)) * error_z/d
                        
                msg_data = Float64()
                msg_data.data = sqrt(pow(agent.x-error_x,2) + pow(agent.y-error_y,2) + pow(agent.z-error_z,2))
                agent.publisher_data_.publish(msg_data)
            
            if not self.continuous:
                delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
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

            self.target_pose.pose.position.x = self.gt_pose.position.x + dx
            self.target_pose.pose.position.y = self.gt_pose.position.y + dy
            self.target_pose.pose.position.z = self.gt_pose.position.z + dz

            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = tf_transformations.euler_from_quaternion((self.gt_pose.orientation.x, self.gt_pose.orientation.y, self.gt_pose.orientation.z, self.gt_pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

            self.node.get_logger().debug('Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (self.gt_pose.position.x, self.target_pose.pose.position.x, self.gt_pose.position.y, self.target_pose.pose.position.y, self.gt_pose.position.z, self.target_pose.pose.position.z)) 
            if self.target_pose.pose.position.z < 0.4:
                self.target_pose.pose.position.z = 0.4

            if self.target_pose.pose.position.z > 2.0:
                self.target_pose.pose.position.z = 2.0

    def pose_pid_controller(self):
        if self.formation_bool:
            ex = ey = ez = 0
            for agent in self.agent_list:
                error_x = agent.x - (self.gt_pose.position.x - agent.pose.position.x)
                error_y = agent.y - (self.gt_pose.position.y - agent.pose.position.y)
                error_z = agent.z - (self.gt_pose.position.z - agent.pose.position.z)
                ex += agent.k * error_x
                ey += agent.k * error_y
                ez += agent.k * error_z
                        
                msg_data = Float64()
                msg_data.data = sqrt(pow(error_x,2)+pow(error_y,2)+pow(error_z,2))
                agent.publisher_data_.publish(msg_data)

            aux = self.node.get_clock().now().to_msg()
            time = aux.sec + aux.nanosec*1e-9
            
            msg = Bool()
            msg.data = True
            dx = dy = dz = 0
            # X Controller
            if self.formation_x_controller.eval_threshold(0.0, ex) or self.continuous:
                self.formation_x_controller.error[0] = ex
                dtx = time - self.formation_x_controller.past_time
                dx = self.formation_x_controller.update(dtx)
                self.formation_x_controller.past_time = time
                self.event_x.publish(msg)
            
            # Y Controller
            if self.formation_y_controller.eval_threshold(0.0, ey) or self.continuous:
                self.formation_y_controller.error[0] = ey
                dty = time - self.formation_y_controller.past_time
                dy = self.formation_y_controller.update(dty)
                self.formation_y_controller.past_time = time
                self.event_y.publish(msg)

            # Z Controller
            if self.formation_z_controller.eval_threshold(0.0, ez) or self.continuous:
                self.formation_z_controller.error[0] = ez
                dtz = time - self.formation_z_controller.past_time
                dz = self.formation_z_controller.update(dtz)
                self.formation_z_controller.past_time = time
                self.event_z.publish(msg)

            self.target_pose.pose.position.x = self.gt_pose.position.x + dx
            self.target_pose.pose.position.y = self.gt_pose.position.y + dy
            self.target_pose.pose.position.z = self.gt_pose.position.z + dz

            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = tf_transformations.euler_from_quaternion((self.gt_pose.orientation.x, self.gt_pose.orientation.y, self.gt_pose.orientation.z, self.gt_pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

            self.node.get_logger().debug('Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (self.gt_pose.position.x, self.target_pose.pose.position.x, self.gt_pose.position.y, self.target_pose.pose.position.y, self.gt_pose.position.z, self.target_pose.pose.position.z)) 
            if self.target_pose.pose.position.z < 0.4:
                self.target_pose.pose.position.z = 0.4

            if self.target_pose.pose.position.z > 2.0:
                self.target_pose.pose.position.z = 2.0

    def eval_threshold(self, signal, ref):
        '''
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
        '''
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
    
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        dt = self.robot.getTime() - self.past_time

        if self.first_pos:
            self.past_x_global = self.gps.getValues()[0]
            self.target_pose.pose.position.x = self.gps.getValues()[0]
            self.past_y_global = self.gps.getValues()[1]
            self.target_pose.pose.position.y = self.gps.getValues()[1]
            self.first_pos = False
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
            self.pose_publisher.publish(init_pose)
            self.z_controller.past_time = self.past_time
            self.w_controller.past_time = self.past_time
            # self.take_off()

        ## Get measurements
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        self.global_yaw = yaw
        roll_rate = self.gyro.getValues()[0]
        pitch_rate = self.gyro.getValues()[1]
        yaw_rate = self.gyro.getValues()[2]
        x_global = self.gps.getValues()[0] # - self.first_x_global
        vx_global = (x_global - self.past_x_global)/dt
        y_global = self.gps.getValues()[1] # - self.first_y_global
        vy_global = (y_global - self.past_y_global)/dt
        z_global = self.gps.getValues()[2]
        vz_global = (z_global - self.past_z_global)/dt

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        self.gt_pose.position.x = x_global
        self.gt_pose.position.y = y_global
        self.gt_pose.position.z = z_global
        self.gt_pose.orientation.x = q[0]
        self.gt_pose.orientation.y = q[1]
        self.gt_pose.orientation.z = q[2]
        self.gt_pose.orientation.w = q[3]

        t_base = TransformStamped()
        t_base.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        t_base.header.frame_id = 'map'
        if self.digital_twin:
            base_name = self.name_value+'_dt/base_link'
        else:
            base_name = self.name_value+'/base_link'
        t_base.child_frame_id = base_name
        t_base.transform.translation.x = x_global
        t_base.transform.translation.y = y_global
        t_base.transform.translation.z = z_global
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tfbr.sendTransform(t_base)
        
        delta = np.array([self.gt_pose.position.x-self.last_pose.position.x,self.gt_pose.position.y-self.last_pose.position.y,self.gt_pose.position.z-self.last_pose.position.z])
  
        if self.communication or np.linalg.norm(delta)>0.01:
            PoseStamp = PoseStamped()
            PoseStamp.pose.position.x = self.gt_pose.position.x
            PoseStamp.pose.position.y = self.gt_pose.position.y
            PoseStamp.pose.position.z = self.gt_pose.position.z
            PoseStamp.pose.orientation.x = self.gt_pose.orientation.x
            PoseStamp.pose.orientation.y = self.gt_pose.orientation.y
            PoseStamp.pose.orientation.z = self.gt_pose.orientation.z
            PoseStamp.pose.orientation.w = self.gt_pose.orientation.w
            PoseStamp.header.frame_id = "map"
            PoseStamp.header.stamp = self.node.get_clock().now().to_msg()
            if self.path_enable:
                self.path.header.stamp = self.node.get_clock().now().to_msg()
                self.path.poses.append(PoseStamp)
                self.path_publisher.publish(self.path)
            self.pose_publisher.publish(PoseStamp)

            self.last_pose.position.x = self.gt_pose.position.x
            self.last_pose.position.y = self.gt_pose.position.y
            self.last_pose.position.z = self.gt_pose.position.z

        ## Position Controller
        # Z Controller
        if self.z_controller.eval_threshold(z_global, self.target_pose.pose.position.z) or self.continuous:
            self.z_controller.error[0] = (self.target_pose.pose.position.z - z_global)
            dtz = self.robot.getTime() - self.z_controller.past_time
            w_ref = self.z_controller.update(dtz)
            self.z_controller.past_time = self.robot.getTime()
            msg = Bool()
            msg.data = True
            self.event_z_.publish(msg)
            self.node.get_logger().debug('Z Controller Event. Th: %.4f; Inc: %.4f; dT: %.4f' % (self.z_controller.th, self.z_controller.inc, dtz))
        else:
            w_ref = self.z_controller.last_value
        self.node.get_logger().debug('Z: R: %.2f P: %.2f C: %.2f' % (self.target_pose.pose.position.z, z_global, w_ref))

        if self.w_controller.eval_threshold(vz_global, w_ref) or self.continuous:
            self.w_controller.error[0] = (w_ref - vz_global)
            dtw = self.robot.getTime() - self.w_controller.past_time
            cmd_thrust = self.w_controller.update(dtw)*1000+38000
            self.w_controller.past_time = self.robot.getTime()
            self.node.get_logger().debug('W Controller Event.V: %.2f; dT: %.3f' % (cmd_thrust, dtw))
        else:
            cmd_thrust = self.w_controller.last_value*1000+38000
        self.node.get_logger().debug('dZ: R: %.2f P: %.2f C: %.2f' % (w_ref, vz_global, cmd_thrust))

        # X-Y Controller
        # IPC Controller 
        if self.controller_IPC:
            self.target_twist = self.IPC_controller()
            # Save zone
            delta = np.array([self.gt_pose.position.x-self.target_pose.pose.position.x,self.gt_pose.position.y-self.target_pose.pose.position.y])
            if np.linalg.norm(delta) < 0.02:
                self.target_twist.angular.z = 0.0
                self.node.get_logger().info('Save zone')
                # X Controller
                if self.x_controller.eval_threshold(x_global, self.target_pose.pose.position.x) or self.continuous:
                    self.x_controller.error[0] = self.target_pose.pose.position.x - x_global
                    dtx = self.robot.getTime() - self.x_controller.past_time
                    self.target_twist.linear.x = self.x_controller.update(dtx)
                    self.x_controller.past_time = self.robot.getTime()
                    msg = Bool()
                    msg.data = True
                    self.event_x_.publish(msg)
                else:
                    self.target_twist.linear.x = self.x_controller.last_value
                self.node.get_logger().debug('X: R: %.2f P: %.2f C: %.2f' % (self.target_pose.pose.position.x, x_global, self.target_twist.linear.x))
                # Y Controller
                if self.y_controller.eval_threshold(y_global, self.target_pose.pose.position.y) or self.continuous:
                    self.y_controller.error[0] = self.target_pose.pose.position.y - y_global
                    dty = self.robot.getTime() - self.y_controller.past_time
                    self.target_twist.linear.y = self.y_controller.update(dty)
                    self.y_controller.past_time = self.robot.getTime()
                    msg = Bool()
                    msg.data = True
                    self.event_y_.publish(msg)
                else:
                    self.target_twist.linear.y = self.y_controller.last_value
                self.node.get_logger().debug('Y: R: %.2f P: %.2f C: %.2f' % (self.target_pose.pose.position.y, y_global, self.target_twist.linear.y))

        if self.controller_PID:    
            # X Controller
            if self.x_controller.eval_threshold(x_global, self.target_pose.pose.position.x) or self.continuous:
                self.x_controller.error[0] = self.target_pose.pose.position.x - x_global
                dtx = self.robot.getTime() - self.x_controller.past_time
                self.target_twist.linear.x = self.x_controller.update(dtx)
                self.x_controller.past_time = self.robot.getTime()
                msg = Bool()
                msg.data = True
                self.event_x_.publish(msg)
            else:
                self.target_twist.linear.x = self.x_controller.last_value
            self.node.get_logger().debug('X: R: %.2f P: %.2f C: %.2f' % (self.target_pose.pose.position.x, x_global, self.target_twist.linear.x))
            # Y Controller
            if self.y_controller.eval_threshold(y_global, self.target_pose.pose.position.y) or self.continuous:
                self.y_controller.error[0] = self.target_pose.pose.position.y - y_global
                dty = self.robot.getTime() - self.y_controller.past_time
                self.target_twist.linear.y = self.y_controller.update(dty)
                self.y_controller.past_time = self.robot.getTime()
                msg = Bool()
                msg.data = True
                self.event_y_.publish(msg)
            else:
                self.target_twist.linear.y = self.y_controller.last_value
            self.node.get_logger().debug('Y: R: %.2f P: %.2f C: %.2f' % (self.target_pose.pose.position.y, y_global, self.target_twist.linear.y))
        
        # dX-dY Controller
        if self.u_controller.eval_threshold(vx_global, self.target_twist.linear.x) or self.continuous:
            self.u_controller.error[0] =  (self.target_twist.linear.x - vx_global)*cos(yaw) + (self.target_twist.linear.y - vy_global)*sin(yaw)
            dtu = self.robot.getTime() - self.u_controller.past_time
            pitch_ref = self.u_controller.update(dtu)
            self.u_controller.past_time = self.robot.getTime()
        else:
            pitch_ref = self.u_controller.last_value
        self.v_controller.error[0] = -(self.target_twist.linear.x - vx_global)*sin(yaw) + (self.target_twist.linear.y - vy_global)*cos(yaw)
        dtv = self.robot.getTime() - self.v_controller.past_time
        roll_ref = self.v_controller.update(dtv)
        self.v_controller.past_time = self.robot.getTime()
        
        ## Attitude Controller
        # Pitch Controller
        self.pitch_controller.error[0] = pitch_ref - degrees(pitch)
        dpitch_ref = self.pitch_controller.update(dt)
        # Roll Controller
        self.roll_controller.error[0] = roll_ref - degrees(roll)
        droll_ref = self.roll_controller.update(dt)
        # Yaw Controller
        angles = tf_transformations.euler_from_quaternion((self.target_pose.pose.orientation.x, self.target_pose.pose.orientation.y, self.target_pose.pose.orientation.z, self.target_pose.pose.orientation.w))
        self.yaw_controller.error[0] = angles[2] - yaw
        if self.yaw_controller.error[0]>pi:
            self.yaw_controller.error[0] = self.yaw_controller.error[0] - 2*pi
        if self.yaw_controller.error[0]<-pi:
            self.yaw_controller.error[0] = self.yaw_controller.error[0] + 2*pi
        dyaw_ref = self.yaw_controller.update(dt)
        # IPC Controller TEST
        # dyaw_ref = self.target_twist.angular.z

        ## Rate Controller
        self.dpitch_controller.error[0] = dpitch_ref - degrees(pitch_rate)
        delta_pitch = self.dpitch_controller.update(dt)
        self.droll_controller.error[0] = droll_ref - degrees(roll_rate)
        delta_roll = self.droll_controller.update(dt)
        self.dyaw_controller.error[0] = dyaw_ref - degrees(yaw_rate)
        delta_yaw = self.dyaw_controller.update(dt)
        
        self.node.get_logger().debug('IPC:: V: %.4f W: %.4f' % (self.target_twist.linear.x, dyaw_ref))

        ## Motor mixing Controller
        motorPower_m1 =  (cmd_thrust - 0.5 * delta_roll - 0.5 * delta_pitch + delta_yaw)
        motorPower_m2 =  (cmd_thrust - 0.5 * delta_roll + 0.5 * delta_pitch - delta_yaw)
        motorPower_m3 =  (cmd_thrust + 0.5 * delta_roll + 0.5 * delta_pitch + delta_yaw)
        motorPower_m4 =  (cmd_thrust + 0.5 * delta_roll - 0.5 * delta_pitch - delta_yaw)

        self.node.get_logger().debug('Thrust(C): %.2f CRoll(C): %.2f CPitch(C): %.2f CYaw(C): %.2f' % (cmd_thrust, delta_roll, delta_pitch, delta_yaw))

        scaling = 1000 ## TO-DO, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(-motorPower_m1/scaling)
        self.m2_motor.setVelocity(motorPower_m2/scaling)
        self.m3_motor.setVelocity(-motorPower_m3/scaling)
        self.m4_motor.setVelocity(motorPower_m4/scaling)

        self.past_time = self.robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global
        self.past_z_global = z_global

    def IPC_controller(self):
        Vmax = 0.5
        K1 = 0.005
        Kp = 1.0
        Ki = 0.008

        d = sqrt(pow(self.target_pose.pose.position.x-self.gt_pose.position.x,2)+pow(self.target_pose.pose.position.y-self.gt_pose.position.y,2))*100
        alpha = atan2(self.target_pose.pose.position.y-self.gt_pose.position.y,self.target_pose.pose.position.x-self.gt_pose.position.x)
        oc = alpha - self.global_yaw
        eo = atan2(sin(oc),cos(oc))
        p = (3.14-abs(eo))/3.14
        V = min(K1*d*p,Vmax)

        self.eomas = eo+self.eomas
        w = Kp*sin(eo) + Ki*self.eomas*0.003

        ## Cmd_Vel
        out = Twist()
        out.linear.x = V * cos(self.global_yaw)
        out.linear.y =  V * sin(self.global_yaw)
        out.angular.z = w

        return out 