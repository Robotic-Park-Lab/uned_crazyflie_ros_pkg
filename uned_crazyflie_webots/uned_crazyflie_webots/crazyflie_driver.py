from cv2 import sqrt
import rclpy
import os
from rclpy.node import Node
from rclpy.time import Time
from threading import Timer

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from math import cos, sin, degrees, radians, pi
import sys
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Change this path to your crazyflie-firmware folder
sys.path.append('/home/kiko/Code/crazyflie-firmware')
import cffirmware

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

        self.target_twist = Twist()
        self.target_pose = Pose()
        self.target_pose.position.z = 0.0

        ## Initialize Sensors
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
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_z_global = 0
        self.past_time = self.robot.getTime()
        self._is_flying = False

        self.first_pos = True
        self.first_x_global = 0.0
        self.first_y_global = 0.0

        ## Intialize Controllers
        self.continuous = False
        # Position
        self.z_controller = PIDController(2.0, 0.50, 0.0, 0.0, 100, 1.0, -1.0, 0.2, 0.01)
        self.x_controller = PIDController(1.0, 0.25, 0.0, 0.0, 100, 1.0, -1.0, 0.2, 0.01)
        self.y_controller = PIDController(1.0, 0.25, 0.0, 0.0, 100, 1.0, -1.0, 0.2, 0.01)
        # Velocity
        self.w_controller = PIDController( 25.0, 15.0, 0.0, 0.0, 100, 26.0, -16.0, 0.2, 0.01)
        self.u_controller = PIDController( 15.0,  0.5, 0.0, 0.0, 100, 30.0, -30.0, 0.2, 0.01)
        self.v_controller = PIDController(-15.0,  0.5, 0.0, 0.0, 100, 30.0, -30.0, 0.2, 0.01)
        # Attitude
        self.pitch_controller = PIDController(6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0, 0.2, 0.01)
        self.roll_controller  = PIDController(6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0, 0.2, 0.01)
        self.yaw_controller   = PIDController(6.0, 1.0, 0.349, 0.0581, 100, 400.0, -400.0, 0.2, 0.01)
        # Rate
        self.dpitch_controller = PIDController(250.0, 500.0,   2.5, 0.01, 100, 0.0, -0.0, 0.2, 0.01)
        self.droll_controller  = PIDController(250.0, 500.0,   2.5, 0.01, 100, 0.0, -0.0, 0.2, 0.01)
        self.dyaw_controller   = PIDController(120.0,  16.698, 0.0, 0.00, 100, 0.0, -0.0, 0.2, 0.01)

        cffirmware.controllerPidInit()

        self.name_value = os.environ['WEBOTS_ROBOT_NAME']
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.name_value+'_driver')
        self.node.get_logger().info('Webots_Node::inicialize() ok. %s' % (str(self.name_value)))
        self.node.create_subscription(Twist, self.name_value+'/cmd_vel', self.cmd_vel_callback, 1)
        self.node.create_subscription(Pose, self.name_value+'/goal_pose', self.goal_pose_callback, 1)
        self.node.create_subscription(String, self.name_value+'/order', self.order_callback, 1)
        self.laser_publisher = self.node.create_publisher(LaserScan, self.name_value+'/scan', 10)
        self.event_x_ = self.node.create_publisher(Bool, self.name_value+'/event_x', 10)
        self.event_y_ = self.node.create_publisher(Bool, self.name_value+'/event_y', 10)
        self.event_z_ = self.node.create_publisher(Bool, self.name_value+'/event_z', 10)
        self.odom_publisher = self.node.create_publisher(Odometry, self.name_value+'/odom', 10)

        # self.tfbr = TransformBroadcaster(self.node)

        self.msg_laser = LaserScan()
        self.node.create_timer(1.0/30.0, self.publish_laserscan_data)
        
    def publish_laserscan_data(self):

        front_range = self.range_front.getValue()/1000.0
        back_range = self.range_back.getValue()/1000.0
        left_range = self.range_left.getValue()/1000.0
        right_range = self.range_right.getValue()/1000.0
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
        self.msg_laser.header.frame_id = 'base_link'
        self.msg_laser.range_min = 0.1
        self.msg_laser.range_max = max_range
        self.msg_laser.ranges = [back_range, left_range, front_range, right_range, back_range]
        self.msg_laser.angle_min = 0.5 * 2*pi
        self.msg_laser.angle_max =  -0.5 * 2*pi
        self.msg_laser.angle_increment = -1.0*pi/2
        self.laser_publisher.publish(self.msg_laser)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def goal_pose_callback(self, pose):
        self.target_pose = pose

    def order_callback(self, msg):
        self.node.get_logger().info('In progress ...')
        self.node.get_logger().info('Order: "%s"' % msg.data)
        if msg.data == 'take_off':
            if self._is_flying:
                self.node.get_logger().warning('Already flying')
            else:
                self.take_off()
        elif msg.data == 'land':
            if self._is_flying:
                self.descent()
            else:
                self.node.get_logger().warning('In land')
        else:
            self.node.get_logger().error('"%s": Unknown order' % (msg.data))
    
    def take_off(self):
        self.node.get_logger().info('Take Off...')
        self.target_pose.position.z = 1.0
        self._is_flying = True
        self.t_ready = Timer(2, self._ready)
        self.t_ready.start()

    def _ready(self):
        self.node.get_logger().info('Ready!!.')
        self.ready = True

    def descent(self):
        msg = Pose()
        self.target_pose.position.z = 0.1
        self.node.get_logger().info('Descent...')
        self.t_desc = Timer(2, self.take_land)
        self._is_flying = False
        self.ready = False
        self.t_desc.start()

    def take_land(self):
        self.node.get_logger().info('Take Land.')
        self.target_pose.position.z = 0.0
        self.first_pos = False

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        dt = self.robot.getTime() - self.past_time

        if self.first_pos is True:
            self.past_x_global = self.gps.getValues()[0]
            self.target_pose.position.x = self.gps.getValues()[0]
            self.past_y_global = self.gps.getValues()[1]
            self.target_pose.position.y = self.gps.getValues()[1]
            self.first_pos = False
            self.z_controller.past_time = self.past_time
            self.w_controller.past_time = self.past_time
            self.take_off()

        ## Get measurements
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        roll_rate = self.gyro.getValues()[0]
        pitch_rate = self.gyro.getValues()[1]
        yaw_rate = self.gyro.getValues()[2]
        x_global = self.gps.getValues()[0] # - self.first_x_global
        vx_global = (x_global - self.past_x_global)/dt
        y_global = self.gps.getValues()[1] # - self.first_y_global
        vy_global = (y_global - self.past_y_global)/dt
        z_global = self.gps.getValues()[2]
        vz_global = (z_global - self.past_z_global)/dt

        ## Position Controller
        # Z Controller
        if self.z_controller.eval_threshold(z_global, self.target_pose.position.z) or self.continuous:
            self.z_controller.error[0] = (self.target_pose.position.z - z_global)
            dtz = self.robot.getTime() - self.z_controller.past_time
            w_ref = self.z_controller.update(dtz)
            self.z_controller.past_time = self.robot.getTime()
            msg = Bool()
            msg.data = True
            self.event_z_.publish(msg)
            # self.node.get_logger().warn('Z Controller Event. Th: %.4f; Inc: %.4f; dT: %.4f' % (self.z_controller.th, self.z_controller.inc, dtz))
        else:
            w_ref = self.z_controller.last_value
        if self.w_controller.eval_threshold(vz_global, w_ref) or self.continuous:
            self.w_controller.error[0] = (w_ref - vz_global)
            dtw = self.robot.getTime() - self.w_controller.past_time
            cmd_thrust = self.w_controller.update(dtw)*1000+38000
            self.w_controller.past_time = self.robot.getTime()
            # self.node.get_logger().info('W Controller Event.V: %.2f; dT: %.3f' % (cmd_thrust, dtw))
        else:
            cmd_thrust = self.w_controller.last_value*1000+38000
        
        # X-Y Controller
        if self.x_controller.eval_threshold(x_global, self.target_pose.position.x) or self.continuous:
            self.x_controller.error[0] = self.target_pose.position.x - x_global
            dtx = self.robot.getTime() - self.x_controller.past_time
            u_ref = self.x_controller.update(dtx)
            self.x_controller.past_time = self.robot.getTime()
            msg = Bool()
            msg.data = True
            self.event_x_.publish(msg)
            # self.node.get_logger().warn('X Controller Event. Th: %.4f; Inc: %.4f; dT: %.4f' % (self.x_controller.th, self.x_controller.inc, dtx))
            # self.target_twist.linear.x = u_ref
        else:
            u_ref = self.x_controller.last_value

        if self.y_controller.eval_threshold(y_global, self.target_pose.position.y) or self.continuous:
            self.y_controller.error[0] = self.target_pose.position.y - y_global
            dty = self.robot.getTime() - self.y_controller.past_time
            v_ref = self.y_controller.update(dty)
            self.y_controller.past_time = self.robot.getTime()
            msg = Bool()
            msg.data = True
            self.event_y_.publish(msg)
        else:
            v_ref = self.y_controller.last_value

        # dX-dY Controller
        if self.u_controller.eval_threshold(vx_global, u_ref) or self.continuous:
            self.u_controller.error[0] =  (u_ref - vx_global)*cos(yaw) + (v_ref - vy_global)*sin(yaw)
            dtu = self.robot.getTime() - self.u_controller.past_time
            pitch_ref = self.u_controller.update(dtu)
            self.u_controller.past_time = self.robot.getTime()
        else:
            pitch_ref = self.u_controller.last_value

        self.v_controller.error[0] = -(u_ref - vx_global)*sin(yaw) + (v_ref - vy_global)*cos(yaw)
        roll_ref = self.v_controller.update(dt)
        
        ## Attitude Controller
        # Pitch Controller
        self.pitch_controller.error[0] = pitch_ref - degrees(pitch)
        dpitch_ref = self.pitch_controller.update(dt)
        # Roll Controller
        self.roll_controller.error[0] = roll_ref - degrees(roll)
        droll_ref = self.roll_controller.update(dt)
        # Yaw Controller
        self.yaw_controller.error[0] = 0.0 - degrees(yaw)
        dyaw_ref = self.yaw_controller.update(dt)

        ## Rate Controller
        self.dpitch_controller.error[0] = dpitch_ref - degrees(pitch_rate)
        delta_pitch = self.dpitch_controller.update(dt)
        self.droll_controller.error[0] = droll_ref - degrees(roll_rate)
        delta_roll = self.droll_controller.update(dt)
        self.dyaw_controller.error[0] = dyaw_ref - degrees(yaw_rate)
        delta_yaw = self.dyaw_controller.update(dt)
        

        q_base = tf_transformations.quaternion_from_euler(0, 0, yaw)
        odom = Odometry()
        odom.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x_global
        odom.pose.pose.position.y = y_global
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = sin(yaw / 2)
        odom.pose.pose.orientation.w = cos(yaw / 2)

        self.odom_publisher.publish(odom)

        t_base = TransformStamped()
        t_base.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = x_global
        t_base.transform.translation.y = y_global
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.z = sin(yaw / 2)
        t_base.transform.rotation.w = cos(yaw / 2)
        # self.tfbr.sendTransform(t_base)

        ## Put measurement in state estimate
        # TODO replace these with a EKF python binding
        state = cffirmware.state_t()
        state.attitude.roll = degrees(roll)
        state.attitude.pitch = -degrees(pitch)
        state.attitude.yaw = degrees(yaw)
        state.position.x = x_global
        state.position.y = y_global
        state.position.z = z_global
        state.velocity.x = vx_global
        state.velocity.y = vy_global
        state.velocity.z = vz_global

        # Put gyro in sensor data
        sensors = cffirmware.sensorData_t()
        sensors.gyro.x = degrees(roll_rate)
        sensors.gyro.y = degrees(pitch_rate)
        sensors.gyro.z = degrees(yaw_rate)
        yawDesired=0

        ## Fill in Setpoints
        setpoint = cffirmware.setpoint_t()
        setpoint.mode.z = cffirmware.modeAbs # 
        setpoint.position.z = self.target_pose.position.z
        setpoint.mode.yaw = cffirmware.modeVelocity
        # TODO: find out why this multipication is necessary...
        setpoint.attitudeRate.yaw = degrees(self.target_twist.angular.z)*5
        setpoint.mode.x = cffirmware.modeVelocity
        setpoint.mode.y = cffirmware.modeVelocity
        # setpoint.position.x = self.target_pose.position.x
        # setpoint.position.y = self.target_pose.position.y
        setpoint.velocity.x = self.target_twist.linear.x
        setpoint.velocity.y = self.target_twist.linear.y
        setpoint.velocity_body = True

        ## Firmware PID bindings
        control = cffirmware.control_t()
        tick = 100 #this value makes sure that the position controller and attitude controller are always always initiated
        cffirmware.controllerPid(control, setpoint,sensors,state,tick)

        ## 
        cmd_roll = radians(control.roll)
        cmd_pitch = radians(control.pitch)
        cmd_yaw = -radians(control.yaw)
        thrust = control.thrust
        # self.node.get_logger().debug('Thrust(F): %.2f CRoll(F): %.2f CPitch(F): %.2f CYaw(F): %.2f' % (thrust, cmd_roll, cmd_pitch, cmd_yaw))

        ## Motor mixing
        motorPower_m1 =  thrust - cmd_roll + cmd_pitch + cmd_yaw
        motorPower_m2 =  thrust - cmd_roll - cmd_pitch - cmd_yaw
        motorPower_m3 =  thrust + cmd_roll - cmd_pitch + cmd_yaw
        motorPower_m4 =  thrust + cmd_roll + cmd_pitch - cmd_yaw

        ## Motor mixing Controller
        motorPower_m1 =  (cmd_thrust - 0.5 * delta_roll - 0.5 * delta_pitch + cmd_yaw)
        motorPower_m2 =  (cmd_thrust - 0.5 * delta_roll + 0.5 * delta_pitch - cmd_yaw)
        motorPower_m3 =  (cmd_thrust + 0.5 * delta_roll + 0.5 * delta_pitch + cmd_yaw)
        motorPower_m4 =  (cmd_thrust + 0.5 * delta_roll - 0.5 * delta_pitch - cmd_yaw)

        self.node.get_logger().debug('Thrust(C): %.2f CRoll(C): %.2f CPitch(C): %.2f CYaw(C): %.2f' % (cmd_thrust, delta_roll, delta_pitch, delta_yaw))
            
        

        # self.node.get_logger().info('Z: %.2f vX: %.2f eX: %.2f vY: %f eY: %.3f Thrust: %.2f' % (z_global,self.target_twist.linear.x, self.x_controller.error[0], self.target_twist.linear.y, self.y_controller.error[0], cmd_thrust))

        scaling = 1000 ##Todo, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(-motorPower_m1/scaling)
        self.m2_motor.setVelocity(motorPower_m2/scaling)
        self.m3_motor.setVelocity(-motorPower_m3/scaling)
        self.m4_motor.setVelocity(motorPower_m4/scaling)

        self.past_time = self.robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global
        self.past_z_global = z_global
