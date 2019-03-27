#!/usr/bin/env python
import rospy
import tf
import csv
import os

FILE_PATH = "/home/aibot/ros_ws/src/"
FILE_NAME = "datafile.csv"

if __name__ == '__main__':
    rospy.init_node('transform_listener')
    listener = tf.TransformListener()

    if os.path.exists(FILE_PATH + FILE_NAME):
        w = csv.writer(open('datafile.csv', 'a'))
    else:
        w = csv.writer(open('datafile.csv', 'a'))
        w.writerow(['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw'])

    data_collected = 0
    while data_collected != 1:
        try:
            (trans, rot) = listener.lookupTransform('/base', '/left_gripper', rospy.Time(0))
            w.writerow(trans + rot)
            print('---------------------')
            print('Data has been written to the file specified')
            print('Position: {}'.format(trans))
            print('Rotation: {}'.format(rot))
            print('---------------------')
            data_collected = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
