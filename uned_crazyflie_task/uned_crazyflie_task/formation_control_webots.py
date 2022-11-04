from dis import dis
import logging
import time
import rclpy
from threading import Timer
import numpy as np
from math import atan2, cos, sin, sqrt

from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, UInt16, UInt16MultiArray, Float64
from geometry_msgs.msg import Pose, Twist, PointStamped, Point
from visualization_msgs.msg import Marker


# List of URIs, comment the one you do not want to fly
uris = set()
dron = list()
publisher = set()

############################
## CF Swarm Logging Class ##
############################
class WebotsAgent:
    def __init__(self, parent, link_uri, ctrl_mode, ctrl_type, role):
        self.role = role
        self.parent = parent
        self.id = 'dron' + link_uri[-2:]
        self.ready = False
        self.pose = Pose()
        self.last_pose = Pose()
        self.agent_list = list()
        self.x_error = 0
        self.y_error = 0
        self.z_error = 0
        self.integral_x = 0
        self.integral_y = 0
        self.integral_z = 0
        self.parent.get_logger().info('%s role: %s!' % (self.id, self.role))
        self._connected()
        self.CONTROL_MODE = ctrl_mode
        self.parent.get_logger().info('CF%s::Control Mode: %s!' % (self.id[-2:], self.CONTROL_MODE))
        self.CONTROLLER_TYPE = ctrl_type
        self.parent.get_logger().info('CF%s::Controller Type: %s!' % (self.id[-2:], self.CONTROLLER_TYPE))

    def _connected(self):
        self.parent.get_logger().info('Connected to %s -> Crazyflie %s' % (self.id, self.id[-2:]))
        # ROS
        # Publisher
        self.publisher_pose = self.parent.create_publisher(Pose, self.id + '/goal_pose', 10)
        self.publisher_order = self.parent.create_publisher(String, self.id + '/order', 10)
        # Subscription
        self.sub_pose = self.parent.create_subscription(Pose, self.id + '/pose', self.newpose_callback, 10)
        
        self._is_flying = False
        self.init_pose = False

    def take_off(self):
        self.t_ready = Timer(2, self._ready)
        self.t_ready.start()

    def _ready(self):
        self.ready = True
        self.parent.get_logger().info('CF%s::Ready!!' % (self.id[-2:]))

    def order_callback(self, msg):
        self.parent.get_logger().info('CF%s::Order: "%s"' % (self.id[-2:], msg.data))
        self.publisher_order.publish(msg)

    def newpose_callback(self, msg):
        self.pose = msg

    def goalpose_callback(self, msg):
        self.publisher_pose.publish(msg)
        self.parent.get_logger().debug('CF%s::New Goal pose: X: %.2f, Y: %.2f, Z: %.2f' % (self.id[-2:], msg.position.x, msg.position.y, msg.position.z))

#####################
## CF Swarm Class  ##
#####################
class CFSwarmWebotsDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('cf_first_uri', 'radio://0/80/2M/E7E7E7E701')
        self.declare_parameter('cf_num_uri', 1)
        self.declare_parameter('cf_controller_type', 'EventBased')
        self.declare_parameter('cf_control_mode', 'HighLevel')
        self.declare_parameter('cf_constrains_mode', 'vector_distance')
        self.declare_parameter('cf_role', 'leader, follower')

        self.publisher_status = self.create_publisher(String,'swarm/status', 10)
        # Subscription
        self.sub_order = self.create_subscription(String, 'swarm/cf_order', self.order_callback, 10)
        self.sub_goal_pose = self.create_subscription(Pose, 'swarm/goal_pose', self.goalpose_callback, 10)

        self.timer_task = self.create_timer(0.02, self.task_manager)
        self.initialize()

    def initialize(self):
        self.get_logger().info('Formation Control::inicialize() ok.')
        # Read Params
        dron_id = self.get_parameter('cf_first_uri').get_parameter_value().string_value
        n = self.get_parameter('cf_num_uri').get_parameter_value().integer_value
        aux = self.get_parameter('cf_control_mode').get_parameter_value().string_value
        control_mode = aux.split(', ')
        aux = self.get_parameter('cf_controller_type').get_parameter_value().string_value
        controller_type = aux.split(', ')
        aux = self.get_parameter('cf_role').get_parameter_value().string_value
        roles = aux.split(', ')
        self.constrains = self.get_parameter('cf_constrains_mode').get_parameter_value().string_value

        # Define crazyflie URIs
        id_address = dron_id[-10:]
        id_base = dron_id[:16]
        id_address_int = int(id_address, 16)
        for i in range(int(n),0,-1):
            cf_str = id_base + hex(id_address_int+i-1)[-10:].upper()
            uris.add(cf_str)
            self.get_logger().warn('Crazyflie %d URI: %s!' % (i, cf_str))
            
            cf = WebotsAgent(self, cf_str, control_mode[i-1], controller_type[i-1], roles[i-1])

            dron.append(cf)
        self.get_logger().info('Formation Control::inicialized.')

    def order_callback(self, msg):
        self.get_logger().info('SWARM::Order: "%s"' % msg.data)
        if msg.data == 'take_off':
            for cf in dron:
                cf.order_callback(msg)
                cf.take_off()
        elif msg.data == 'distance_formation_run':
            for cf in dron:
                cf.order_callback(msg)
            self._ready()
        else:
            for cf in dron:
                cf.order_callback(msg)

    def _ready(self):
        self.get_logger().info('SWARM::Ready!!')
        msg = String()
        msg.data = 'Ready'
        self.publisher_status.publish(msg)

    def goalpose_callback(self, msg):
        self.get_logger().info('SWARM::New Goal pose: In progress ...')
        '''
        for cf in dron:
            cf.cmd_motion_.x = cf.cmd_motion_.x + msg.position.x
            cf.cmd_motion_.y = cf.cmd_motion_.y + msg.position.y
            cf.cmd_motion_.z = cf.cmd_motion_.z + msg.position.z
            cf.cmd_motion_.ckeck_pose()
            delta = [abs(msg.position.x), abs(msg.position.y), abs(msg.position.z)]
            self.cmd_motion_.flight_time = max(delta)/self.max_vel
            cf.cmd_motion_.send_pose_data_(cf.scf.cf)

        self.get_logger().info('SWARM::New Goal pose: X:%0.2f \tY:%0.2f \tZ:%0.2f' % (msg.position.x, msg.position.y, msg.position.z))
        '''

    def task_manager(self):
        for cf in dron:
            if cf.ready and len(cf.agent_list)>0 and False:
                msg = Pose()
                msg.position.x = cf.pose.position.x
                msg.position.y = cf.pose.position.y
                msg.position.z = cf.pose.position.z
                dx = dy = dz = 0
                for agent in cf.agent_list:
                    error_x = cf.pose.position.x - agent.pose.position.x
                    error_y = cf.pose.position.y - agent.pose.position.y
                    error_z = cf.pose.position.z - agent.pose.position.z
                    if self.constrains == 'distance_hover':
                        # alfa = atan2(error_y, error_x)
                        # beta = atan2(error_z, error_x)
                        distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                        dx += (pow(agent.d,2) - distance) * error_x
                        dy += (pow(agent.d,2) - distance) * error_y
                        dz += (pow(agent.d,2) - distance) * error_z
                        msg_data = Float64()
                        msg_data.data = agent.d - sqrt(distance)
                        agent.publisher_data.publish(msg_data)
                        # self.get_logger().warn('Main: %s ID: %s D: %.3f dx: %.2f dy: %.2f dz: %.2f' % (cf.id, agent.id, distance, dx, dy, dz))
                    else:
                        dx += agent.x - error_x
                        dy += agent.y - error_y
                        dz += agent.z - error_z
                msg.position.x += (dx/4)
                msg.position.y += (dy/4)
                msg.position.z += (dz/4)
                
                if msg.position.z < 0.5:
                    msg.position.z = 0.5

                if msg.position.z > 1.2:
                    msg.position.z = 1.2

                
                # self.get_logger().info('dx: %f dy: %f dz: %f' % (dx, dy, dz))
                # self.get_logger().error('DX: %f DY: %f DZ: %f' % (msg.position.x, msg.position.y, msg.position.z))

                # TO-DO: Integral Term
                # cf.integral_x += (1/(len(cf.agent_list)*1.0))*cf.x_error*0.02
                # cf.integral_y += (1/(len(cf.agent_list)*1.0))*cf.y_error*0.02
                # cf.integral_z += (1/(len(cf.agent_list)*5.0))*cf.z_error*0.02
                # msg.position.x += (dx/len(cf.agent_list)) + cf.integral_x
                # msg.position.y += (dy/len(cf.agent_list)) + cf.integral_y
                # msg.position.z += (dz/len(cf.agent_list)) + cf.integral_z
                # cf.x_error = dx
                # cf.y_error = dy
                # cf.z_error = dz

                cf.goalpose_callback(msg)

                


def main(args=None):
    rclpy.init(args=args)
    swarm_driver = CFSwarmWebotsDriver()
    rclpy.spin(swarm_driver)

    swarm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
