# -*- coding: utf-8 -*-
# https://github.com/ros2/rosbag2/issues/473

import sqlite3
import csv
import sys
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import matplotlib.pyplot as plt

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]



if __name__ == "__main__":

        bag_file = sys.argv[1]
        parser = BagFileParser(bag_file)

        # CF_data
        data = parser.get_messages("/cf_data")[:][:]
        #[position][0: time; 1: data]

        header = ['Timestamp', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw', 'Thrust']
        data_csv = []
        for i in range(len(parser.get_messages("/cf_data"))):
            aux = [(data[i][1].timestamp - data[0][1].timestamp)/1000,
                   data[i][1].x, data[i][1].y, data[i][1].z,
                   data[i][1].roll, data[i][1].pitch, data[i][1].yaw,
                   data[i][1].thrust]
            data_csv.append(aux)

        aux = bag_file.split('/')
        aux.pop()
        dir = ''.join(aux)+'/cd_data.csv'
        with open(dir, 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(data_csv)

        # Vicon data
        data = parser.get_messages("/dron01/pose")[:][:]
        header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
        data_csv = []
        for i in range(len(parser.get_messages("/dron01/pose"))):
            aux = [(data[i][0] - data[0][0])*1e-10,
                   data[i][1].position.x, data[i][1].position.y, data[i][1].position.z,
                   data[i][1].orientation.x, data[i][1].orientation.y, data[i][1].orientation.z, data[i][1].orientation.w]
            data_csv.append(aux)

        aux = bag_file.split('/')
        aux.pop()
        dir = ''.join(aux)+'/dron_pose.csv'
        with open(dir, 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(data_csv)
