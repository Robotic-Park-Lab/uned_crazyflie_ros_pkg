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
end_test = False
is_deck_attached = False
position_estimate = [0, 0]
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

class CFDriver(Node):
    def __init__(self):
        super().__init__('cf_driver')

        # timer_period = 0.1  # seconds
        self.publisher_ = self.create_publisher(StateEstimate, 'cf_data', 10)
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.iterate_loop = self.create_timer(timer_period, self.iterate)
        cflib.crtp.init_drivers()
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            self.initialize(scf)
            if is_deck_attached:
                self.take_off_simple(scf)
            while not end_test:
                time.sleep(0.5)
            # self.iterate()

    def initialize(self, scf):
        self.get_logger().info('CrazyflieDriver::inicialize() ok.')
        self.log_async(scf)

    def log_async(self, scf):
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)
        cf = scf.cf
        self.get_logger().info('Connected to %s' % uri)
        scf.cf.disconnected.add_callback(self.disconnected)

        self.lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        self.lg_stab.add_variable('stateEstimate.x', 'float')
        self.lg_stab.add_variable('stateEstimate.y', 'float')
        self.lg_stab.add_variable('stateEstimate.z', 'float')
        self.lg_stab.add_variable('stabilizer.roll', 'float')
        self.lg_stab.add_variable('stabilizer.pitch', 'float')
        self.lg_stab.add_variable('stabilizer.yaw', 'float')
        self.lg_stab.add_variable('pm.vbat', 'FP16')

        cf.log.add_config(self.lg_stab)

        self.lg_stab.data_received_cb.add_callback(self.stab_log_data)
        # This callback will be called on errors
        self.lg_stab.error_cb.add_callback(self.stab_log_error)
        # Start the logging
        self.lg_stab.start()

    def stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        self.get_logger().info('Error when logging %s: %s'
                               % (logconf.name, msg))

    def stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        self.data_callback(timestamp, data)
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()

    def disconnected(self, uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        global end_test
        print('Disconnected from %s' % uri)
        end_test = True

    def data_callback(self, timestamp, data):
        msg = StateEstimate()
        msg.timestamp = timestamp
        msg.x = data['stateEstimate.x']
        msg.y = data['stateEstimate.y']
        msg.z = data['stateEstimate.z']
        self.publisher_.publish(msg)



    def take_off_simple(self, scf):
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            time.sleep(1)
            mc.stop()


def main(args=None):
    rclpy.init(args=args)
    cf_driver = CFDriver()

    cf_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
