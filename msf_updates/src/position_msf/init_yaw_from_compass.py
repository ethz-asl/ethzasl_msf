#!/usr/bin/env python

PACKAGE = 'msf_updates'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf
from math import *

import dynamic_reconfigure.client

heading = 0.0
is_init_heading = False

# Specify MSF node name parameter from launch file
client = dynamic_reconfigure.client.Client(rospy.get_param("msf_sensor_node",
    "msf_gps_pose_estimator/position_sensor"), timeout=30.0)

def msg_to_quaternion(msg):
        q = [0,0,0,1]
        q[0] = msg.x
        q[1] = msg.y
        q[2] = msg.z
        q[3] = msg.w
        return q

# Write init_yaw MSF parameter from orientation message
def callback_imu(data):
    global is_init_heading
    global heading
    radians = tf.transformations.euler_from_quaternion(msg_to_quaternion(data.orientation))
    heading = degrees(radians[2])
    is_init_heading = True

def callback_orientation_degrees(data):
    global is_init_heading
    global heading
    heading = data.z
    is_init_heading = True

def callback_orientation_quaternion(data):
    global is_init_heading
    global heading
    # This call will also probably need msg_to_quaternion() to convert data into suitable format for transformation.py library

    radians = tf.transformations.euler_from_quaternion(data)
    heading = degrees(radians[2])
    is_init_heading = True

def callback_heading_radians(data):
    global is_init_heading
    global heading
    heading = degrees(data.data)
    is_init_heading = True

if __name__ == "__main__":
    rospy.init_node("yaw_init_dyn_reconfigure_client")

    sub_imu = rospy.Subscriber('imu', Imu,
        callback_imu, queue_size=1, tcp_nodelay=True)
    sub_orientation_quaternion = rospy.Subscriber('orientation_quaternion', Quaternion,
        callback_orientation_quaternion, queue_size=1, tcp_nodelay=True)
    sub_orientation_degrees = rospy.Subscriber('orientation_degrees', Vector3,
        callback_orientation_degrees, queue_size=1, tcp_nodelay=True)
    sub_heading_radians = rospy.Subscriber('heading_radians', Float64,
        callback_heading_radians, queue_size=1, tcp_nodelay=True)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown() and not is_init_heading:
        r.sleep()

    while not rospy.is_shutdown():
        print rospy.get_param("msf_sensor_node",
            "msf_gps_pose_estimator/position_sensor")
        client.update_configuration({"position_yaw_init":heading})
        r.sleep()
