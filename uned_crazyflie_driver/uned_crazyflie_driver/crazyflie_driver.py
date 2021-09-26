import logging
import time
import rclpy
from threading import Timer
from rclpy.node import Node
from std_msgs.msg import String
from uned_crazyflie_config.msg import StateEstimate

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
DEFAULT_HEIGHT = 0.5
is_deck_attached = False
position_estimate = [0, 0]
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class Logging:
    def __init__(self, link_uri, parent):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')
        self.parent = parent
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.parent.get_logger().info('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        self.parent.get_logger().info('Connected to %s' % link_uri)
        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable('pm.vbat', 'FP16')
        """
        Groups:
        - range: front(uint16), back(uint16), up(uint16), left(uint16),
                 right(uint16), zranger(uint16)
        - motor: m1(uint32), m2(uint32), m3(uint32), m4(uint32)
        - stabilizer: estimator, controller[Type], stop,
                      roll(float), pith(float), yaw(float), thrust(float)
        - ctrltarget:  x(float),  y(float),  z(float),
                      vx(float), vy(float), vz(float),
                      ax(float), ay(float), az(float),
                      roll(float), pitch(float), yaw(float)
        - acc:  x(float),  y(float),  z(float),
        - accSec:  x(float),  y(float),  z(float), [IMU]
        https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/
        """
        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
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
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        # t = Timer(10, self._cf.close_link)
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        self.parent.get_logger().info('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        self.parent.data_callback(timestamp, data)
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False
        self.parent.destroy_node()
        rclpy.shutdown()

class CFDriver(Node):
    def __init__(self):
        super().__init__('cf_driver')
        self.initialize()
        timer_period = 0.1  # seconds
        self.publisher_ = self.create_publisher(StateEstimate, 'cf_data', 10)
        self.take_off_simple(self.CF)
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.iterate_loop = self.create_timer(timer_period, self.iterate)

    def data_callback(self, timestamp, data):
        msg = StateEstimate()
        msg.timestamp = timestamp
        msg.x = data['stateEstimate.x']
        msg.y = data['stateEstimate.y']
        msg.z = data['stateEstimate.z']
        self.publisher_.publish(msg)

    def initialize(self):
        self.get_logger().info('CrazyflieDriver::inicialize() ok.')
        cflib.crtp.init_drivers()
        self.CF = Logging(uri, self)
        #


    def take_off_simple(self, scf):
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            time.sleep(3)
            mc.stop()

def main(args=None):
    rclpy.init(args=args)
    cf_driver = CFDriver()
    rclpy.spin(cf_driver)

    cf_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
