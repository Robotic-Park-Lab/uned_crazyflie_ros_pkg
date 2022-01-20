import logging
import time
import rclpy
from threading import Timer
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from uned_crazyflie_config.msg import StateEstimate
from uned_crazyflie_config.msg import Pidcontroller
from vicon_receiver.msg import Position

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# List of URIs, comment the one you do not want to fly
uris = set()
dron = list()
publisher = set()

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
        self.parent.get_logger().info('Connected to %s -> Crazyflie %s' % (link_uri, self.link_uri[-2:]))
        # ROS
        id = 'dron' + self.link_uri[-2:]
        self.publisher_pose = self.parent.create_publisher(Pose, id + '/cf_pose', 10)
        self.publisher_twist = self.parent.create_publisher(Twist, id + '/cf_twist', 10)
        # POSE3D
        self._lg_stab_pose = LogConfig(name='Pose', period_in_ms=10)
        self._lg_stab_pose.add_variable('stateEstimate.x', 'float')
        self._lg_stab_pose.add_variable('stateEstimate.y', 'float')
        self._lg_stab_pose.add_variable('stateEstimate.z', 'float')
        self._lg_stab_pose.add_variable('stabilizer.roll', 'float')
        self._lg_stab_pose.add_variable('stabilizer.pitch', 'float')
        self._lg_stab_pose.add_variable('stabilizer.yaw', 'float')
        # TWIST
        self._lg_stab_twist = LogConfig(name='Twist', period_in_ms=10)
        self._lg_stab_twist.add_variable('gyro.x', 'float')
        self._lg_stab_twist.add_variable('gyro.y', 'float')
        self._lg_stab_twist.add_variable('gyro.z', 'float')
        self._lg_stab_twist.add_variable('stateEstimate.vx', 'float')
        self._lg_stab_twist.add_variable('stateEstimate.vy', 'float')
        self._lg_stab_twist.add_variable('stateEstimate.vz', 'float')
        # Other data. TO-DO
        self.parent.get_logger().warning('Crazyflie %s. TO-DO: Add _lg_stab_data' % self.link_uri[-2:])
        # self._lg_stab_data = LogConfig(name='Data', period_in_ms=10)
        # self._lg_stab_data.add_variable('controller.cmd_thrust', 'float')
        # self._lg_stab_data.add_variable('pm.vbat', 'FP16')
        self.scf.cf.param.add_update_callback(group='posCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='velCtlPid', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='pid_attitude', cb=self.param_stab_est_callback)
        self.scf.cf.param.add_update_callback(group='pid_rate', cb=self.param_stab_est_callback)
        # self.scf.cf.param.add_update_callback(group='deck', cb=self.param_stab_est_callback)
        try:
            self.scf.cf.log.add_config(self._lg_stab_pose)
            self.scf.cf.log.add_config(self._lg_stab_twist)
            # This callback will receive the data
            self._lg_stab_pose.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab_twist.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab_pose.error_cb.add_callback(self._stab_log_error)
            self._lg_stab_twist.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab_pose.start()
            self._lg_stab_twist.start()
        except KeyError as e:
            self.parent.get_logger().info('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.link_uri[-2:])

    def _stab_log_error(self, logconf, msg):
        self.parent.get_logger().error('Crazyflie %s. Error when logging %s: %s' % (self.link_uri[-2:], logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        if(logconf.name == "Pose"):
            self.pose_callback(timestamp, data)
        elif(logconf.name == "Twist"):
            self.twist_callback(timestamp, data)
        else:
            self.parent.get_logger().error('Crazyflie %s. Error: %s: not valid logconf' % (self.link_uri[-2:], logconf.name))
        # elif (logconf.name == "Data"):
        #     self.parent.data_callback(timestamp, data, self.link_uri[-2:])

    def param_stab_est_callback(self, name, value):
        self.parent.get_logger().info('Crazyflie %s. Parameter %s: %s' %(self.link_uri[-2:], name, value))

    def _connection_failed(self, link_uri, msg):
        self.parent.get_logger().error('Crazyflie %s. Connection to %s failed: %s' % (self.link_uri[-2:], link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        self.parent.get_logger().error('Crazyflie %s. Connection to %s lost: %s' % (self.link_uri[-2:], link_uri, msg))

    def _disconnected(self, link_uri):
        self.parent.get_logger().warning('Crazyflie %s. Disconnected from %s' % (self.link_uri[-2:], link_uri))
        self.is_connected = False

    def pose_callback(self, timestamp, data):
        msg = Pose()
        msg.position.x = data['stateEstimate.x']
        msg.position.y = data['stateEstimate.y']
        msg.position.z = data['stateEstimate.z']
        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']
        msg.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        msg.orientation.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        msg.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        msg.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        self.publisher_pose.publish(msg)

    def twist_callback(self, timestamp, data, n):
        msg = Twist()
        msg.linear.x = data['stateEstimate.vx']
        msg.linear.y = data['stateEstimate.vy']
        msg.linear.z = data['stateEstimate.vz']
        msg.angular.x = data['gyro.x']
        msg.angular.y = data['gyro.y']
        msg.angular.z = data['gyro.z']

        self.publisher_twist.publish(msg)


#####################
## CF Swarm Class  ##
#####################
class CFSwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('cf_first_uri', 'radio://0/80/2M/E7E7E7E701')
        self.declare_parameter('cf_num_uri', 1)

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
        self.get_logger().debug('SwarmDriver::iterate() ok.')



def main(args=None):
    rclpy.init(args=args)
    swarm_driver = CFSwarmDriver()
    rclpy.spin(swarm_driver)

    cf_driver.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
