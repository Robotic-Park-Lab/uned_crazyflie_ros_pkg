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
dron = list()

############################
## CF Swarm Logging Class ##
############################
class CFLogging:
    def __init__(self, scf, parent, link_uri):
        self.parent = parent
        self.parent.get_logger().info('Connecting to %s' % link_uri)
        self.scf = scf
        self.link_uri = link_uri
        self.scf.cf.connected.add_callback(self._connected)
        self.scf.cf.disconnected.add_callback(self._disconnected)
        self.scf.cf.connection_failed.add_callback(self._connection_failed)
        self.scf.cf.connection_lost.add_callback(self._connection_lost)

        self.scf.cf.open_link(link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        self.parent.get_logger().info('Connected to %s' % link_uri)
        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        # self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('controller.cmd_thrust', 'float')
        # The fetch-as argument can be set to FP16 to save space
        # in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        try:
            self.scf.cf.log.add_config(self._lg_stab)
            # crazyflie.log.add_config([logconf1, logconfig2])
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            self.parent.get_logger().info('Could not add Stabilizer log config, bad configuration.')

    def _stab_log_error(self, logconf, msg):
        self.parent.get_logger().info('Error when logging %s: %s'
                                      % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        self.parent.data_callback(timestamp, data, self.link_uri[-2:])

    def _connection_failed(self, link_uri, msg):
        self.parent.get_logger().info('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        self.parent.get_logger().info('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        self.parent.get_logger().info('Disconnected from %s' % link_uri)
        self.is_connected = False

#####################
## CF Swarm Class  ##
#####################
class CFSwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('cf_first_uri', 'radio://0/80/2M/E7E7E7E701')
        self.declare_parameter('cf_num_uri', 2)
        # Publisher
        self.publisher01_ = self.create_publisher(StateEstimate, 'dron01/cf_data', 10)
        self.publisher02_ = self.create_publisher(StateEstimate, 'dron02/cf_data', 10)
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
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        self.cf_swarm = Swarm(uris, factory=factory)
        for uri in uris:
            cf = CFLogging(self.cf_swarm._cfs[uri], self, uri)
            dron.append(cf)


    def iterate(self):
        self.get_logger().info('SwarmDriver::iterate() ok.')

    def data_callback(self, timestamp, data, n):
        msg = StateEstimate()
        msg.timestamp = timestamp
        msg.x = data['stateEstimate.x']
        msg.y = data['stateEstimate.y']
        msg.z = data['stateEstimate.z']
        msg.roll = data['stabilizer.roll']
        msg.pitch = data['stabilizer.pitch']
        # msg.yaw = data['stabilizer.yaw']
        msg.thrust = data['controller.cmd_thrust']
        if n == '01':
            self.publisher01_.publish(msg)
        if n == '02':
            self.publisher02_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    swarm_driver = CFSwarmDriver()
    rclpy.spin(swarm_driver)

    cf_driver.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
