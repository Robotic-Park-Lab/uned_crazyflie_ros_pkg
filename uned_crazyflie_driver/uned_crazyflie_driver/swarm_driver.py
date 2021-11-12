import logging
import time
import rclpy
from threading import Timer

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from uned_crazyflie_config.msg import StateEstimate
from uned_crazyflie_config.msg import Cmdsignal

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger

URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E702'

# List of URIs, comment the one you do not want to fly
uris = set()

#####################
## CF Swarm Class
#####################
class CFSwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('cf_first_uri', 'radio://0/80/2M/E7E7E7E701')
        self.declare_parameter('cf_num_uri', 2)
        # Publisher

        # Subscription

        timer_period = 0.1
        self.iterate_loop = self.create_timer(timer_period, self.iterate)
        self.initialize()

    def initialize(self):
        self.get_logger().info('SwarmDriver::inicialize() ok.')
        # Read Params
        dron_id = self.get_parameter('cf_first_uri').get_parameter_value().string_value
        n = self.get_parameter('cf_num_uri').get_parameter_value().integer_value
        # Define crazyflie URIs
        uris.add(dron_id)
        id_address = dron_id[-10:]
        id_base = dron_id[:16]
        id_address_int = int(id_address, 16)
        self.get_logger().info('Crazyflie 1 URI: %s!' % dron_id)
        for i in range(1,int(n)):
            cf_str = id_base + hex(id_address_int+i)[-10:].upper()
            uris.add(cf_str)
            self.get_logger().info('Crazyflie %d URI: %s!' % (i+1, cf_str))


        # logging.basicConfig(level=logging.DEBUG)
        # cflib.crtp.init_drivers()
        # factory = CachedCfFactory(rw_cache='./cache')

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
