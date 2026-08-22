# -*- coding: utf-8 -*-
# Conversor genérico de un fichero ros2 bag (sqlite3, formato por defecto de
# ROS 2 Humble) a un .csv por topic, basado en el tipo de mensaje en vez de
# en una lista fija de nombres de topic -- así convierte cualquier bag de
# este repo (uned_crazyflie_driver, uned_crazyflie_webots, uned_crazyflie_task,
# uned_crazyflie_controllers...) sin tener que mantener una lista de topics
# a mano. Adaptado de
# https://github.com/Robotic-Park-Lab/RoboticPark/blob/main/scripts/bag2csv_benchmark.py
# con dos tipos de mensaje añadidos, propios de este repo: std_msgs/Float64MultiArray
# (onboard_cmd) y std_msgs/UInt16MultiArray (cf_data).
#
# Uso: python3 bag2csv.py <carpeta_del_bag>
# (la carpeta debe contener <carpeta_del_bag>_0.db3 y metadata.yaml, que es
# lo que genera `ros2 bag record` por defecto)

import sqlite3
import csv
import sys
import os
import yaml
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {
            name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

    def __del__(self):
        self.conn.close()

    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(
                topic_id)).fetchall()
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name]))
                for timestamp, data in rows]


def write_csv(out_dir, topic_name, header, rows):
    path = os.path.join(out_dir, topic_name[1:].replace('/', '-') + '.csv')
    with open(path, 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print('Uso: python3 bag2csv.py <carpeta_del_bag>')
        sys.exit(1)

    bag_dir = sys.argv[1]
    bag_file = os.path.join(bag_dir, os.path.basename(bag_dir.rstrip('/')) + '_0.db3')
    parser = BagFileParser(bag_file)

    metadata_file = os.path.join(bag_dir, 'metadata.yaml')
    with open(metadata_file, 'r') as file:
        topics_info = yaml.load(file, Loader=yaml.FullLoader) \
            .get('rosbag2_bagfile_information').get('topics_with_message_count')

    for entry in topics_info:
        name = entry['topic_metadata'].get('name')
        msg_type = entry['topic_metadata'].get('type')
        data = parser.get_messages(name)

        if msg_type in ("geometry_msgs/msg/Pose",):
            print('Converting topic: ' + name + ' ...')
            header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
            rows = [[t, m.position.x, m.position.y, m.position.z,
                     m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]
                    for t, m in data]
        elif msg_type == "geometry_msgs/msg/PoseStamped":
            print('Converting topic: ' + name + ' ...')
            header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
            rows = [[t, m.pose.position.x, m.pose.position.y, m.pose.position.z,
                     m.pose.orientation.x, m.pose.orientation.y,
                     m.pose.orientation.z, m.pose.orientation.w]
                    for t, m in data]
        elif msg_type in ("std_msgs/msg/Float64", "std_msgs/msg/Float32",
                           "std_msgs/msg/String"):
            print('Converting topic: ' + name + ' ...')
            header = ['Timestamp', 'Data']
            rows = [[t, m.data] for t, m in data]
        elif msg_type == "geometry_msgs/msg/Twist":
            print('Converting topic: ' + name + ' ...')
            header = ['Timestamp', 'VX', 'VY', 'VZ', 'WX', 'WY', 'WZ']
            rows = [[t, m.linear.x, m.linear.y, m.linear.z,
                     m.angular.x, m.angular.y, m.angular.z] for t, m in data]
        elif msg_type == "geometry_msgs/msg/PointStamped":
            print('Converting topic: ' + name + ' ...')
            header = ['Timestamp', 'X', 'Y', 'Z']
            rows = [[t, m.point.x, m.point.y, m.point.z] for t, m in data]
        elif msg_type == "geometry_msgs/msg/Vector3":
            print('Converting topic: ' + name + ' ...')
            header = ['Timestamp', 'X', 'Y', 'Z']
            rows = [[t, m.x, m.y, m.z] for t, m in data]
        elif msg_type == "std_msgs/msg/Float64MultiArray":
            # uned_crazyflie_driver/onboard_cmd: [Thrust, Roll, Pitch, Yaw]
            print('Converting topic: ' + name + ' ...')
            header = ['Timestamp', 'Data']
            rows = [[t, list(m.data)] for t, m in data]
        elif msg_type == "std_msgs/msg/UInt16MultiArray":
            # uned_crazyflie_driver/cf_data: contador de eventos [X, Y, Z]
            print('Converting topic: ' + name + ' ...')
            header = ['Timestamp', 'Data']
            rows = [[t, list(m.data)] for t, m in data]
        elif msg_type == "rosgraph_msgs/msg/Clock":
            print('Converting topic: ' + name + ' ...')
            header = ['Timestamp', 'Sec', 'NanoSec']
            rows = [[t, m.clock.sec, m.clock.nanosec] for t, m in data]
        else:
            continue

        try:
            write_csv(bag_dir, name, header, rows)
        except Exception as e:
            print('No se pudo escribir el csv de %s: %s' % (name, e))
