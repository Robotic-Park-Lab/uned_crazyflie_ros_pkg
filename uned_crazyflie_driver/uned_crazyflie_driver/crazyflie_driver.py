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
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
CONTROL_MODE = 'Null'
goal_pose = Pose()
goal_pose.position.x = 0.0
goal_pose.position.y = 0.0
"""
Test:
HighLevel: Trajectory
OffBoard: Trajectory + Position
LowLevel: Trajectory + Position + Attitude
"""
DEFAULT_HEIGHT = 1.0
end_test = False
is_deck_attached = False
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class CMD_Motion():
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0
        self.thrust = 0

    def str_(self):
        return ('Thrust: ' + str(self.thrust) + ' Roll: ' + str(self.roll) +
                ' Pitch: ' + str(self.pitch)+' Yaw: ' + str(self.yaw))
#


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

    def _stab_log_error(self, logconf, msg):
        self.parent.get_logger().info('Error when logging %s: %s'
                                      % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        self.parent.data_callback(timestamp, data)
        '''
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
           print(f'{name}: {value:3.3f} ', end='')
        print()
        '''

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self.is_connected = False
        rclpy.shutdown()


#


class CFDriver(Node):
    def __init__(self):
        super().__init__('cf_driver')

        timer_period = 0.01  # seconds
        self.publisher_ = self.create_publisher(StateEstimate, 'cf_data', 10)

        self.sub_order = self.create_subscription(String, 'cf_order',
                                                  self.order_callback, 10)
        self.sub_pose = self.create_subscription(Pose, 'dron01/pose',
                                                 self.new_pose, 10)
        self.sub_cmd = self.create_subscription(Cmdsignal, 'cf_cmd_control',
                                                self.cmd_control_callback, 10)

        self.iterate_loop = self.create_timer(timer_period, self.iterate)
        cflib.crtp.init_drivers()
        self.initialize()
        self.despegue()
        t = Timer(5, self.descenso)
        t.start()

    def initialize(self):
        self.get_logger().info('CrazyflieDriver::inicialize() ok.')
        self.scf = Logging(uri, self)
        self.cmd_motion_ = CMD_Motion()
        self.scf._cf.commander.set_client_xmode(True)
        time.sleep(2.0)
        # Init Kalman Filter
        self.scf._cf.param.set_value('stabilizer.estimator', '2')
        # Set the std deviation for the quaternion data pushed into the
        # kalman filter. The default value seems to be a bit too low.
        self.scf._cf.param.set_value('locSrv.extQuatStdDev', 0.06)
        # Reset Estimator
        self.scf._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.scf._cf.param.set_value('kalman.resetEstimation', '0')
        # Init Move
        self.scf._cf.commander.send_setpoint(0.0, 0.0, 0, 0)
        # self.mc = MotionCommander(self.scf._cf, default_height=1.0)
        # self.cmd = Commander(self.scf._cf)
        # self.ramp_motors()

    def despegue(self):
        global goal_pose
        goal_pose.position.z = 1.0
        self.get_logger().info('CrazyflieDriver::Despegue.')

    def descenso(self):
        global goal_pose
        goal_pose.position.z = 0.1
        self.get_logger().info('CrazyflieDriver::Descenso.')
        t0 = Timer(2, self.aterrizaje)
        t0.start()

    def aterrizaje(self):
        self.get_logger().info('CrazyflieDriver::Aterrizaje.')
        goal_pose.position.z = 0.0
        self.scf._cf.commander.send_setpoint(0.0, 0.0, 0, 0)
        self.scf._cf.commander.send_stop_setpoint()

    def iterate(self):
        global goal_pose
        # self.get_logger().info(self.cmd_motion_.str_())
        if (goal_pose.position.z > 0.05):
            self.scf._cf.commander.send_position_setpoint(goal_pose.position.x,
                                                          goal_pose.position.y,
                                                          goal_pose.position.z,
                                                          0)

        # self.scf._cf.commander.send_setpoint(self.cmd_motion_.roll, self.cmd_motion_.pitch, self.cmd_motion_.yaw, self.cmd_motion_.thrust)

    def data_callback(self, timestamp, data):
        msg = StateEstimate()
        msg.timestamp = timestamp
        msg.x = data['stateEstimate.x']
        msg.y = data['stateEstimate.y']
        msg.z = data['stateEstimate.z']
        msg.roll = data['stabilizer.roll']
        msg.pitch = data['stabilizer.pitch']
        # msg.yaw = data['stabilizer.yaw']
        msg.thrust = data['controller.cmd_thrust']
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

    def cmd_control_callback(self, msg):
        self.cmd_motion_.roll = msg.roll
        self.cmd_motion_.pitch = msg.pitch
        self.cmd_motion_.yaw = msg.yaw
        self.cmd_motion_.thrust = msg.thrust

    def new_pose(self, msg):
        self.scf._cf.extpos.send_extpose(msg.position.x, msg.position.y,
                                         msg.position.z, msg.orientation.x,
                                         msg.orientation.y, msg.orientation.z,
                                         msg.orientation.w)
#


def main(args=None):
    rclpy.init(args=args)
    cf_driver = CFDriver()
    rclpy.spin(cf_driver)

    cf_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
