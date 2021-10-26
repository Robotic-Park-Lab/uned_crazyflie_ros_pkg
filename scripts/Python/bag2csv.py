# -*- coding: utf-8 -*-
# https://github.com/ros2/rosbag2/issues/473

import sqlite3
import csv
import sys
import yaml
import os
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

        bag_file = sys.argv[1]+'/'+sys.argv[1]+'_0.db3'
        parser = BagFileParser(bag_file)
        aux = bag_file.split('/')
        aux.pop()
        metadata_file = ''.join(aux)+'/metadata.yaml'
        topic_list = []
        with open(metadata_file, 'r') as file:
            topics_file = yaml.load(file, Loader=yaml.FullLoader)
            topics_info = topics_file.get("rosbag2_bagfile_information").get("topics_with_message_count")
            for i in range(len(topics_info)):
                topic_aux = topics_info[i]['topic_metadata'].get("name")
                topic_list.append(topic_aux)
        # print(topic_list)

        # CF_data
        dir_file = sys.argv[1]+'/cf_data.csv'
        if ('/cf_data' in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: /cf_data ...')
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
            dir = ''.join(aux)+'/cf_data.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Vicon data
        dir_file = sys.argv[1]+'/dron_pose.csv'
        if ('/dron01/pose' in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: /dron01/pose ...')
            data = parser.get_messages("/dron01/pose")[:][:]
            header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
            data_csv = []
            for i in range(len(parser.get_messages("/dron01/pose"))):
                aux = [(data[i][0] - data[0][0])*1e-9,
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

        # Cmd_control data
        dir_file = sys.argv[1]+'/cf_cmd_control.csv'
        if ('/cf_cmd_control' in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: /cf_cmd_control ...')
            data = parser.get_messages("/cf_cmd_control")[:][:]
            header = ['Timestamp', 'Thrust', 'Roll', 'Pitch', 'Yaw', 'vBat']
            data_csv = []
            for i in range(len(parser.get_messages("/cf_cmd_control"))):
                aux = [(data[i][0] - data[0][0])*1e-9,
                       data[i][1].thrust, data[i][1].roll, data[i][1].pitch,
                       data[i][1].yaw, data[i][1].vbat]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/cf_cmd_control.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Ref Pose data
        dir_file = sys.argv[1]+'/pose_ref.csv'
        if ('/pose_ref' in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: /pose_ref ...')
            data = parser.get_messages("/pose_ref")[:][:]
            header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
            data_csv = []
            for i in range(len(parser.get_messages("/pose_ref"))):
                aux = [(data[i][0] - data[0][0])*1e-9,
                       data[i][1].position.x, data[i][1].position.y, data[i][1].position.z,
                       data[i][1].orientation.x, data[i][1].orientation.y, data[i][1].orientation.z, data[i][1].orientation.w]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/pose_ref.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)
