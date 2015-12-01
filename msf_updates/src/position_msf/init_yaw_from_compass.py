#!/usr/bin/env python

PACKAGE = 'msf_updates'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float64
import tf
from math import *

import dynamic_reconfigure.client

client = dynamic_reconfigure.client.Client(rospy.get_param("msf_sensor_node",
    "msf_gps_pose_estimator/position_sensor"), timeout=30)

# Write init_yaw MSF parameter from orientation message
def callback_orientation_degrees(data):
    client.update_configuration({"position_yaw_init":data.z})

def callback_orientation_quaternion(data):
    degrees = tf.transformations.euler_from_quaternion(data)
    client.update_configuration({"position_yaw_init":degrees[2]})

def callback_heading_radians(data):
    client.update_configuration({"position_yaw_init":degrees(data.data)})

if __name__ == "__main__":
    rospy.init_node("yaw_init_dyn_reconfigure_client")

    sub_orientation_quaternion = rospy.Subscriber('orientation_quaternion', Quaternion,
        callback_orientation_quaternion, queue_size=1, tcp_nodelay=True)
    sub_orientation_degrees = rospy.Subscriber('orientation_degrees', Vector3,
        callback_orientation_degrees, queue_size=1, tcp_nodelay=True)
    sub_heading_radians = rospy.Subscriber('heading_radians', Float64,
        callback_heading_radians, queue_size=1, tcp_nodelay=True)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        r.sleep()
