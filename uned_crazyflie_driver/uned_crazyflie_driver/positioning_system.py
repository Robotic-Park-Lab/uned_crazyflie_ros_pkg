import time
import rclpy
import numpy as np
import yaml

from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from math import cos, sin, degrees, radians, pi, sqrt

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# List of URIs, comment the one you do not want to fly
uris = set()
dron = list()

######################
## CF Logging Class ##
######################
class Crazyflie_ROS2():
    def __init__(self, parent, scf, link_uri, name, type, config):
        self.scf = scf
        self.parent = parent
        self.id = name
        self.config = config
        
        self.tfbr = TransformBroadcaster(self.parent)
        self.parent.get_logger().info('Connecting to %s: %s' % (name,link_uri))
        self.scf.cf.connected.add_callback(self._connected)
        self.scf.cf.disconnected.add_callback(self._disconnected)
        self.scf.cf.connection_failed.add_callback(self._connection_failed)
        self.scf.cf.connection_lost.add_callback(self._connection_lost)
        self.scf.cf.open_link(link_uri)

        self.scf.CONTROLLER_TYPE = type
        self.parent.get_logger().info('%s::Controller Type: %s!' % (self.id, self.scf.CONTROLLER_TYPE))

    def _connected(self, link_uri):
        self.parent.get_logger().info('Connected to %s ->  %s' % (link_uri, self.id))
        # ROS
        # Publisher
        # POSE3D
        if self.config['local_pose']['enable']:
            self.publisher_pose = self.parent.create_publisher(Pose, self.id + '/local_pose', 10)
            self._lg_stab_pose = LogConfig(name='Pose', period_in_ms=self.config['local_pose']['T'])
            self._lg_stab_pose.add_variable('stateEstimate.x', 'float')
            self._lg_stab_pose.add_variable('stateEstimate.y', 'float')
            self._lg_stab_pose.add_variable('stateEstimate.z', 'float')
            self._lg_stab_pose.add_variable('stabilizer.roll', 'float')
            self._lg_stab_pose.add_variable('stabilizer.pitch', 'float')
            self._lg_stab_pose.add_variable('stabilizer.yaw', 'float')
            try:
                self.scf.cf.log.add_config(self._lg_stab_pose)
                self._lg_stab_pose.data_received_cb.add_callback(self._stab_log_data)
                self._lg_stab_pose.error_cb.add_callback(self._stab_log_error)

                self._lg_stab_pose.start()
            except KeyError as e:
                self.parent.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # TWIST
        if self.config['local_twist']['enable']:
            self.publisher_twist = self.parent.create_publisher(Twist, self.id + '/local_twist', 10)
            self._lg_stab_twist = LogConfig(name='Twist', period_in_ms=self.config['local_twist']['T'])
            self._lg_stab_twist.add_variable('gyro.x', 'float')
            self._lg_stab_twist.add_variable('gyro.y', 'float')
            self._lg_stab_twist.add_variable('gyro.z', 'float')
            self._lg_stab_twist.add_variable('stateEstimate.vx', 'float')
            self._lg_stab_twist.add_variable('stateEstimate.vy', 'float')
            self._lg_stab_twist.add_variable('stateEstimate.vz', 'float')
            try:
                self.scf.cf.log.add_config(self._lg_stab_twist)
                self._lg_stab_twist.data_received_cb.add_callback(self._stab_log_data)
                self._lg_stab_twist.error_cb.add_callback(self._stab_log_error)

                self._lg_stab_twist.start()
            except KeyError as e:
                self.parent.get_logger().info('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.parent.get_logger().error('Crazyflie %s. Could not add Stabilizer log config, bad configuration.' % self.scf.cf.link_uri[-2:])

        # Params
        self.scf.cf.param.add_update_callback(group='deck', cb=self.param_stab_est_callback)

        self.scf.cf.commander.set_client_xmode(True)

    def _stab_log_error(self, logconf, msg):
        self.parent.get_logger().error('%s. Error when logging %s: %s' % (self.id, logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        if(logconf.name == "Pose"):
            self.pose_callback(data)
        elif(logconf.name == "Twist"):
            self.twist_callback(data)
        else:
            self.parent.get_logger().error('%s. Error: %s: not valid logconf' % (self.id, logconf.name))

    def param_stab_est_callback(self, name, value):
        self.parent.get_logger().info('%s. Parameter %s: %s' %(self.id, name, value))

    def _connection_failed(self, link_uri, msg):
        self.parent.get_logger().error('%s. Connection to %s failed: %s' % (self.id, link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        self.parent.get_logger().error('%s. Connection to %s lost: %s' % (self.id, link_uri, msg))

    def _disconnected(self, link_uri):
        self.parent.get_logger().warning('%s. Disconnected from %s' % (self.id, link_uri))
        self.is_connected = False
        self.parent.destroy_node()

    def pose_callback(self, data):
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

        t_base = TransformStamped()
        t_base.header.stamp = self.parent.get_clock().now().to_msg()
        t_base.header.frame_id = 'map'
        t_base.child_frame_id = self.id
        t_base.transform.translation.x = msg.position.x
        t_base.transform.translation.y = msg.position.y
        t_base.transform.translation.z = msg.position.z
        t_base.transform.rotation.x = msg.orientation.x
        t_base.transform.rotation.y = msg.orientation.y
        t_base.transform.rotation.z = msg.orientation.z
        t_base.transform.rotation.w = msg.orientation.w
        self.tfbr.sendTransform(t_base)

    def twist_callback(self, data):
        msg = Twist()
        msg.linear.x = data['stateEstimate.vx']
        msg.linear.y = data['stateEstimate.vy']
        msg.linear.z = data['stateEstimate.vz']
        msg.angular.x = data['gyro.x']
        msg.angular.y = data['gyro.y']
        msg.angular.z = data['gyro.z']

        self.publisher_twist.publish(msg)

#####################
#####################
## CF Swarm Class  ##
#####################
class CFPositioningSystem(Node):
    def __init__(self):
        super().__init__('positioning_system')
        # Params
        self.declare_parameter('first_uri', 'radio://0/80/2M/E7E7E7E701')
        self.declare_parameter('n', 1)
        self.declare_parameter('agents', 'khepera01')
        self.declare_parameter('type', 'EventBased')
        """
        Test:
        Continuous: Continuous
        EventBased: Event Based
        """
        self.declare_parameter('config', 'file_path.yaml')

        self.initialize()

    def initialize(self):
        self.get_logger().info('PositioningSystem::inicialize() ok.')
        self.pose = Pose()
        # Read Params
        dron_id = self.get_parameter('first_uri').get_parameter_value().string_value
        n = self.get_parameter('n').get_parameter_value().integer_value
        aux = self.get_parameter('agents').get_parameter_value().string_value
        names = aux.split(', ')
        aux = self.get_parameter('type').get_parameter_value().string_value
        type = aux.split(', ')
        config_file = self.get_parameter('config').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
            
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
            id = 'dron' + uri[-2:]
            config = documents[id]
            cf = Crazyflie_ROS2(self, self.cf_swarm._cfs[uri], uri, names[i], type[i], config)
            dron.append(cf)
            while not cf.scf.cf.param.is_updated:
                time.sleep(1.0)
            self.get_logger().warning('Parameters downloaded for %s' % cf.scf.cf.link_uri)
            i += 1

        self.cf_swarm.parallel_safe(self.update_params)

    def update_params(self, scf):
        
        # Disable Flow deck to EKF
        scf.cf.param.set_value('motion.disable', '1')
        # Init Kalman Filter
        scf.cf.param.set_value('stabilizer.estimator', '2')
        # Set the std deviation for the quaternion data pushed into the
        # kalman filter. The default value seems to be a bit too low.
        scf.cf.param.set_value('locSrv.extQuatStdDev', 0.06)
        # Reset Estimator
        scf.cf.param.set_value('kalman.resetEstimation', '1')
        scf.cf.param.set_value('kalman.resetEstimation', '0')


def main(args=None):
    rclpy.init(args=args)
    positioning_system_node = CFPositioningSystem()
    rclpy.spin(positioning_system_node)

    positioning_system_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
