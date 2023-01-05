import rclpy
import os
import sys
import tf_transformations

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker

from math import cos, sin, degrees, radians, pi, sqrt
from tf2_ros import TransformBroadcaster

class Agent():
    def __init__(self, parent, id, x = None, y = None, z = None, d = None):
        self.id = id
        self.parent = parent
        if d == None:
            self.x = x
            self.y = y
            self.z = z
            self.parent.get_logger().info('Agent: %s' % self.str_())
        else:
            self.d = d
            self.parent.get_logger().info('Agent: %s' % self.str_distance_())
        self.pose = Pose()
        self.sub_pose_ = self.parent.create_subscription(Pose, self.id + '/local_pose', self.gtpose_callback, 10)
        self.publisher_data_ = self.parent.create_publisher(Float64, self.parent.name_value + '/' + self.id + '/data', 10)
        self.publisher_marker_ = self.parent.create_publisher(Marker, self.parent.name_value + '/' + self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))
    
    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def gtpose_callback(self, msg):
        self.pose = msg

        line = Marker()
        p0 = Point()
        p0.x = self.parent.gt_pose.position.x
        p0.y = self.parent.gt_pose.position.y
        p0.z = self.parent.gt_pose.position.z

        p1 = Point()
        p1.x = self.pose.position.x
        p1.y = self.pose.position.y
        p1.z = self.pose.position.z
        # self.parent.distance_formation_bool = True

        distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
    
        line.header.frame_id = 'map'
        line.header.stamp = self.parent.node.get_clock().now().to_msg()
        line.id = 1
        line.type = 5
        line.action = 0
        line.scale.x = 0.01
        line.scale.y = 0.01
        line.scale.z = 0.01

        if abs(distance - self.d) > 0.05:
            line.color.r = 1.0
        else:
            if abs(distance - self.d) > 0.025:
                line.color.r = 1.0
                line.color.g = 0.5
            else:
                line.color.g = 1.0
        line.color.a = 1.0
        line.points.append(p1)
        line.points.append(p0)

        self.publisher_marker_.publish(line)