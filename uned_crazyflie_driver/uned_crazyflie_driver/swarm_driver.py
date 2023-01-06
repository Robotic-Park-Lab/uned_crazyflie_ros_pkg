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

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from uned_crazyflie_driver.crazyflie_ros_driver import Crazyflie_ROS2

# List of URIs, comment the one you do not want to fly
uris = set()
dron = list()

#####################
## CF Swarm Class  ##
#####################
class CFSwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('cf_first_uri', 'radio://0/80/2M/E7E7E7E701')
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
        Test:
        Continuous: Continuous PID
        EventBased: Event Based PID
        """
        self.publisher_status_ = self.create_publisher(String,'swarm/status', 10)

        # Subscription
        self.sub_order = self.create_subscription(String, 'swarm/order', self.order_callback, 10)
        self.sub_pose_ = self.create_subscription(Pose, 'swarm/pose', self.newpose_callback, 10)
        self.sub_goal_pose_ = self.create_subscription(Pose, 'swarm/goal_pose', self.goalpose_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('SwarmDriver::inicialize() ok.')
        self.pose = Pose()
        # Read Params
        dron_id = self.get_parameter('cf_first_uri').get_parameter_value().string_value
        n = self.get_parameter('cf_num_uri').get_parameter_value().integer_value
        aux = self.get_parameter('cf_control_mode').get_parameter_value().string_value
        control_mode = aux.split(', ')
        aux = self.get_parameter('cf_controller_type').get_parameter_value().string_value
        controller_type = aux.split(', ')

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
            cf = Crazyflie_ROS2(self.cf_swarm._cfs[uri], uri, control_mode[i], controller_type[i])
            dron.append(cf)
            while not cf.scf.cf.param.is_updated:
                time.sleep(1.0)
            self.get_logger().warning('Parameters downloaded for %s' % cf.scf.cf.link_uri)
            i += 1

        self.cf_swarm.parallel_safe(self.update_params)
        for cf in dron:
            rclpy.spin(cf.node)

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
            self.swarm_ready = Timer(2, self._ready)
            self.swarm_ready.start()
        elif msg.data == 'land':
            for cf in dron:
                cf.descent()
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
            #self.cmd_motion_.flight_time = max(delta)/self.max_vel
            self.cmd_motion_.flight_time = 0.5
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
