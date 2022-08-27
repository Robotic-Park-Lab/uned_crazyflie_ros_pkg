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
        print(topic_list)

        # CF_data
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_cf_data.csv'
        topic_name = '/'+sys.argv[2]+'/cf_data'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            #[position][0: time; 1: data]

            header = ['Timestamp', 'X', 'Y', 'Z']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                if (len(data[i][1].data) == 3):
                    aux = [data[i][0], data[i][1].data[0], data[i][1].data[1], data[i][1].data[2]]
                    data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_cf_data.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # CF_pose
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_cf_pose.csv'
        topic_name = '/'+sys.argv[2]+'/cf_pose'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            #[position][0: time; 1: data]

            header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].position.x, data[i][1].position.y, data[i][1].position.z,
                       data[i][1].orientation.x, data[i][1].orientation.y, data[i][1].orientation.z, data[i][1].orientation.w]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_cf_pose.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # CF_twist
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_cf_twist.csv'
        topic_name = '/'+sys.argv[2]+'/cf_twist'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'VX', 'VY', 'VZ', 'WX', 'WY', 'WZ']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].linear.x, data[i][1].linear.y, data[i][1].linear.z,
                       data[i][1].angular.x, data[i][1].angular.y, data[i][1].angular.z]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_cf_twist.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Vicon data
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_pose.csv'
        topic_name = '/'+sys.argv[2]+'/pose'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].position.x, data[i][1].position.y, data[i][1].position.z,
                       data[i][1].orientation.x, data[i][1].orientation.y, data[i][1].orientation.z, data[i][1].orientation.w]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_pose.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Goal Pose data
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_goal_pose.csv'
        topic_name = '/'+sys.argv[2]+'/goal_pose'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].position.x, data[i][1].position.y, data[i][1].position.z,
                       data[i][1].orientation.x, data[i][1].orientation.y, data[i][1].orientation.z, data[i][1].orientation.w]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_goal_pose.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Onboard Cmd
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_onboard_cmd.csv'
        topic_name = '/'+sys.argv[2]+'/onboard_cmd'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'Thrust', 'Roll', 'Pitch', 'Yaw']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].data[0], data[i][1].data[1], data[i][1].data[2],
                       data[i][1].data[3]]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_onboard_cmd.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Order
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_order.csv'
        topic_name = '/'+sys.argv[2]+'/cf_order'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'Data']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].data[0]]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_order.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)
