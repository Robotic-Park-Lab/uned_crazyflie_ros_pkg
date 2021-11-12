import logging
import time
import rclpy
from threading import Timer

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from uned_crazyflie_config.msg import StateEstimate
from uned_crazyflie_config.msg import Cmdsignal


#####################
## CF Swarm Class
#####################
class CFSwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params

        # Publisher

        # Subscription

        timer_period = 0.1
        self.iterate_loop = self.create_timer(timer_period, self.iterate)
        self.initialize()

    def initialize(self):
        self.get_logger().info('SwarmDriver::inicialize() ok.')

    def iterate(self):
        self.get_logger().info('SwarmDriver::iterate() ok.')

def main(args=None):
    rclpy.init(args=args)
    swarm_driver = CFSwarmDriver()
    rclpy.spin(swarm_driver)

    cf_driver.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
