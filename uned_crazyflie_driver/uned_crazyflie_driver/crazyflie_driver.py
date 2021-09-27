import logging
import time
import rclpy
import sys

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
CONTROL_MODE = 'Null'
"""
Null:
Test:
HighLevel:
OffBoard:
LowLevel:
"""
DEFAULT_HEIGHT = 1.0
end_test = False
is_deck_attached = False
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def param_deck_flow(name, value_str):
    value = int(value_str)
    global is_deck_attached
    if value:
        is_deck_attached = True
        print('Flow2 deck is attached!')
    else:
        is_deck_attached = False
        print('Flow2 deck is NOT attached!')
class Logging:
    def __init__(self, link_uri, parent):
        """ Initialize and run the example with the specified link_uri """

        self.cf = Crazyflie(rw_cache='./cache')
        self.parent = parent
        # Connect some callbacks from the Crazyflie API
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)

        self.parent.get_logger().info('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self.cf.open_link(link_uri)

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

        try:
            self.cf.log.add_config(self._lg_stab)
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

    def _stab_log_error(self, logconf, msg):
        self.parent.get_logger().info('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        self.parent.data_callback(timestamp, data)
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self.is_connected = False
        rclpy.shutdown()
"""
"""
class CFDriver(Node):
    def __init__(self):
        super().__init__('cf_driver')

        timer_period = 0.1  # seconds
        self.publisher_ = self.create_publisher(StateEstimate, 'cf_data', 10)
        self.subscription = self.create_subscription(String, 'cf_order', self.order_callback, 10)
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.iterate_loop = self.create_timer(timer_period, self.iterate)
        cflib.crtp.init_drivers()
        self.initialize()
        # self.mc.take_off()
        # t = Timer(2, self.aterrizaje)
        # t.start()


    def initialize(self):
        self.get_logger().info('CrazyflieDriver::inicialize() ok.')
        self.scf = Logging(uri, self)
        time.sleep(1.0)

        self.mc = MotionCommander(self.scf.cf, default_height=DEFAULT_HEIGHT)

    def despegue(self):
        self.mc.take_off()

    def aterrizaje(self):
        self.mc.land()

    def iterate(self):
        self.get_logger().info('In progress ...')

    def data_callback(self, timestamp, data):
        msg = StateEstimate()
        msg.timestamp = timestamp
        msg.x = data['stateEstimate.x']
        msg.y = data['stateEstimate.y']
        msg.z = data['stateEstimate.z']
        self.publisher_.publish(msg)

    def order_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == 'take_off':
            if self.mc._is_flying:
                self.get_logger().info('Already flying')
            else:
                self.despegue()
        if msg.data == 'land':
            if self.mc._is_flying:
                self.aterrizaje()
            else:
                self.get_logger().info('In land')

    def take_off_simple(self, scf):
        self.get_logger().info('Test Mode Control')
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            mc.up(0.3)
            time.sleep(1)
            mc.stop()

    def high_level_control(self):
        self.mc.take_off()
        time.sleep(1)
        self.mc.land()



def main(args=None):
    rclpy.init(args=args)
    cf_driver = CFDriver()
    rclpy.spin(cf_driver)

    cf_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
