# Copyright 2026 Robotic Park Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Robotic Park Lab nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import time
import rclpy
from threading import Timer
import yaml

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm

from uned_crazyflie_driver.crazyflie_agent import Crazyflie_ROS2

# List of URIs, comment the one you do not want to fly
uris = set()
dron = list()


#####################
# CF Swarm Class  ##
#####################
class CFSwarmDriver(Node):
    def __init__(self):
        super().__init__('swarm_driver')
        # Params
        self.declare_parameter('config', 'file_path.yaml')
        self.declare_parameter('robots', '')

        # Publisher
        self.publisher_status_ = self.create_publisher(String, '/swarm/status', 10)
        self.publisher_order = self.create_publisher(String, '/swarm/order', 10)

        # Subscription
        self.sub_order = self.create_subscription(String, '/swarm/order', self.order_callback, 10)
        self.sub_pose_ = self.create_subscription(
            PoseStamped, '/swarm/pose', self.newpose_callback, 10)
        self.sub_goal_pose_ = self.create_subscription(
            Pose, '/swarm/goal_pose', self.goalpose_callback, 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('SwarmDriver::inicialize() ok.')

        self.pose = Pose()
        # Read Params
        config_file = self.get_parameter('config').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)

        # Define crazyflie URIs
        for robot in documents['Robots']:
            if not documents['Robots'][robot]['type'] == 'virtual':
                self.get_logger().info('Crazyflie %s:: %s' % (documents['Robots'][robot]['name'], documents['Robots'][robot]['uri']))
                uris.add(documents['Robots'][robot]['uri'])

        # logging.basicConfig(level=logging.DEBUG)
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        self.cf_swarm = Swarm(uris, factory=factory)

        for robot in documents['Robots']:
            if not documents['Robots'][robot]['type'] == 'virtual':
                config = documents['Robots'][robot]
                id = config['name']
                self.get_logger().info('Crazyflie %s::%s' % (id, config['uri']))

                # cf = Crazyflie_ROS2(self, self.cf_swarm._cfs[config['uri']], config['uri'], id, config)
                cf = Crazyflie_ROS2(self, self, config['uri'], id, config, scf=self.cf_swarm._cfs[config['uri']])
                dron.append(cf)
                # while not cf.scf.cf.param.is_updated:
                #     time.sleep(0.1)
                # self.get_logger().warning('Parameters downloaded for %s' % cf.scf.cf.link_uri)

        time.sleep(1.0)
        self.cf_swarm.parallel_safe(self.swarm_connection)
        time.sleep(1.0)
        self.cf_swarm.parallel_safe(self.update_params)

        for agent in dron:
            agent.led_ring = agent.scf.cf.param.get_value('deck.bcLedRing')
            # agent.load_formation_params()

        msg = String()
        msg.data = 'init'
        # self.publisher_status_.publish(msg)

    def update_params(self, scf):
        # Disable Flow deck to EKF
        # if scf.CONTROL_MODE == 'None' or scf.CONTROL_MODE == 'Gimbal':
        #     scf.cf.param.set_value('motion.disable', '1')
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
        # Multi-Agent Robotic Control
        scf.cf.param.set_value('stabilizer.controller', '5')
        scf.cf.param.set_value('ring.effect', '2')

    def swarm_connection(self, scf):
        self.get_logger().info('Connecting to %s' % scf.uri)
        scf.cf.open_link(scf.uri)
        while not scf.cf.param.is_updated:
            time.sleep(0.1)

    def order_callback(self, msg):
        self.get_logger().info('SWARM::Order: "%s"' % msg.data)
        if msg.data == 'take_off':
            for cf in dron:
                cf.take_off()
            self.swarm_ready = Timer(4, self._ready)
            self.swarm_ready.start()
        elif msg.data == 'land' or msg.data == 'formation_run' or msg.data == 'formation_stop':
            for cf in dron:
                cf.order_callback(msg)
        else:
            self.get_logger().error('SWARM::"%s": Unknown order' % msg.data)

    def stop_dataset(self):
        msg = String()
        msg.data = 'end'
        self.publisher_order.publish(msg)
        self.get_logger().info('Multi-Robot-System::Order: "%s"' % msg.data)

    def _ready(self):
        self.get_logger().info('SWARM::Ready!!')
        msg = String()
        msg.data = 'ready'
        self.publisher_status_.publish(msg)

    def goalpose_callback(self, msg):
        for cf in dron:
            cf.cmd_motion_.x = cf.cmd_motion_.x + msg.position.x
            cf.cmd_motion_.y = cf.cmd_motion_.y + msg.position.y
            cf.cmd_motion_.z = cf.cmd_motion_.z + msg.position.z
            cf.cmd_motion_.ckeck_pose()

            cf.scf.cf.high_level_commander.go_to_target_pose(
                cf.cmd_motion_.x, cf.cmd_motion_.y, cf.cmd_motion_.z)

        self.get_logger().info(
            'SWARM::New Goal pose: X:%0.2f \tY:%0.2f \tZ:%0.2f' %
            (msg.position.x, msg.position.y, msg.position.z))

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
