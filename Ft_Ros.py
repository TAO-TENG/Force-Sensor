#!/usr/bin/python

import time
import os
import socket
import struct
import threading
import numpy as np
from sys import getsizeof
import argparse
from datetime import datetime
# import rofunc as rf
# from rofunc.utils.logger.beauty_logger import beauty_print
import rospy
from geometry_msgs.msg import WrenchStamped
import sys

def connect_and_set_sensor(ip: str, port: int, sample_rate: int, buff_size: int):
    """
    Args:
        ip: ip address
        port: port
        sample_rate:
        buff_size:

    Returns:
        None
    """
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((ip, port))
    print("Connected to socket: {}:{}".format(ip, port))

    set_sam_rate = 'AT+SMPF=' + str(sample_rate) + '\r\n'
    client.send(set_sam_rate.encode('utf-8'))  # 设置采样率
    data_raw = client.recv(buff_size)
    print(data_raw)

    client.send(b'AT+DCKMD=SUM\r\n')  # 设置数据校验方法（sum）
    data_raw = client.recv(buff_size)
    print(data_raw)

    client.send(b'AT+ADJZF=1;1;1;1;1;1\r\n')  # 传感器信号调零
    data_raw = client.recv(buff_size)
    print(data_raw)

    # client.send(b'AT+GSD=\r\n')  # 连续上传数据包
    # client.send(b'AT+GOD=\r\n')  # 只上传一个包数据
    return client


def length_check(data_raw):
    head_index = 0
    if getsizeof(data_raw)/2 > 30:
        for i in range(30):
            if data_raw[i] == 0xAA and data_raw[i + 1] == 0x55:
                head_index = i
        frame_len = data_raw[head_index + 2] * 256 + data_raw[head_index + 3]
        return frame_len
    else:
        return 0


def sum_check(data_raw):
    check = data_raw[30]
    check_new = 0x00
    for i in range(6, 30):
        check_new = check_new + int(data_raw[i])
    if hex(check)[-2:] == hex(check_new)[-2:]:
        return True
    else:
        return False


def read_batch(client, buff_size):

    while not rospy.is_shutdown():
        force_data = []
        data_raw = client.recv(buff_size)
        if not data_raw:
            break
        frame_len = length_check(data_raw)  # 长度校验（27）
        # print(frame_len)
        if frame_len == 27:
            force_sensor_data = struct.unpack('ffffff', data_raw[6:30])  # 依次为六轴力传感器数据
            if sum_check(data_raw) is True:  # 数据校验（sum）
                force_data.append(force_sensor_data)
        return force_data


def publisher(ip: str, port: int, sample_rate: int, buff_size: int):
    """
    Args:
        ip: ip address
        port: port
        sample_rate:
        buff_size:
        label:

    Returns:
        None
    """
    pub = rospy.Publisher('SRI_force_topic_left',WrenchStamped,queue_size=10)
    rate = rospy.Rate(200)
    client = connect_and_set_sensor(ip, port, sample_rate, buff_size)

    print("One FT is working now!!!")


    while not rospy.is_shutdown():

        client.send(b'AT+GOD=\r\n')  # 只上传一个包数据
        force_data = read_batch(client, buff_size)

        if force_data != []:  
            fs = WrenchStamped()
            fs.wrench.force.x = force_data[0][0]
            fs.wrench.force.y = force_data[0][1]
            fs.wrench.force.z = force_data[0][2]
            fs.wrench.torque.x = force_data[0][3]
            fs.wrench.torque.y = force_data[0][4]
            fs.wrench.torque.z = force_data[0][5]
            fs.header.stamp = rospy.Time.now()
            pub.publish(fs)
        rate.sleep()
    client.close()


def publisher2(ip1: str, ip2: str, port: int, sample_rate: int, buff_size: int):
    """
    Args:
        ip: ip address
        port: port
        sample_rate:
        buff_size:
        label:
    Returns:
        None
    """
    pub_1 = rospy.Publisher('SRI_force_topic_left',WrenchStamped,queue_size=10)
    client1 = connect_and_set_sensor(ip1, port, sample_rate, buff_size)

    pub_2 = rospy.Publisher('SRI_force_topic_right',WrenchStamped,queue_size=10)
    client2 = connect_and_set_sensor(ip2, port, sample_rate, buff_size)

    rate = rospy.Rate(200)

    print("Two FT are working now!!!")

    while not rospy.is_shutdown():
        client1.send(b'AT+GOD=\r\n')  # 只上传一个包数据
        client2.send(b'AT+GOD=\r\n')  # 只上传一个包数据
        force_data1 = read_batch(client1, buff_size)
        force_data2 = read_batch(client2, buff_size)

        fs1 = WrenchStamped()
        fs2 = WrenchStamped()

        if force_data1 != [] and force_data2 != [] :  
            fs1.wrench.force.x = force_data1[0][0]
            fs1.wrench.force.y = force_data1[0][1]
            fs1.wrench.force.z = force_data1[0][2]
            fs1.wrench.torque.x = force_data1[0][3]
            fs1.wrench.torque.y = force_data1[0][4]
            fs1.wrench.torque.z = force_data1[0][5]
            fs1.header.stamp = rospy.Time.now()

            fs2.wrench.force.x = force_data2[0][0]
            fs2.wrench.force.y = force_data2[0][1]
            fs2.wrench.force.z = force_data2[0][2]
            fs2.wrench.torque.x = force_data2[0][3]
            fs2.wrench.torque.y = force_data2[0][4]
            fs2.wrench.torque.z = force_data2[0][5]
            fs2.header.stamp = rospy.Time.now()
            pub_1.publish(fs1)
            pub_2.publish(fs2)  
        rate.sleep()
    client1.close()
    client2.close()


def ft_pub(ip1, ip2, port, sample_rate, buff_size=8192):
    """
    Args:
        ip1: IP address of an SRI force sensor.
        ip2: IP address of an SRI force sensor.
        port:
        sample_rate:
        t: number of batches (running time * sample_rate / samples_per_read)
        root_path: root dictionary
        exp_name: dictionary saving the npy file, named according to time
        buff_size:

    Returns: None
    """
    print('Initialize the Publisher!!!')
    if ip2 is not None:

        publisher2(ip1, ip2, port, sample_rate, buff_size=8192)

    else:
        publisher(ip1, port, sample_rate, buff_size=8192)
        # publisher(ip2, port, sample_rate, buff_size=8192)
    print('SRI force sensor stopped')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        '-a', '--addr',
        dest='host1',
        default='192.168.0.111',
        help="IP address of the force sensor 1.")
    parser.add_argument(
        '-a2', '--addr2',
        dest='host2',
        default='192.168.0.110',
        # default=None,
        help="IP address of the force sensor 2.")
    args = parser.parse_args()

    root_dir = '/home/clover/cuhk_ws/src/Fs'
    exp_name = datetime.now().strftime('%Y%m%d_%H%M%S')

    rospy.init_node('SRI_node',anonymous=True)

    try:
        ft_pub(ip1=args.host1, ip2=args.host2, port=4008, sample_rate=150)
    except rospy.ROSInterruptException:
        pass
