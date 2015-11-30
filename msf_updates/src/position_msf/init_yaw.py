#!/usr/bin/env python

PACKAGE = 'msf_updates'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
from geometry_msgs.msg import Vector3

import dynamic_reconfigure.client

client = dynamic_reconfigure.client.Client("msf_gps_pose_estimator/position_sensor", timeout=30)

# Write init_yaw MSF parameter from orientation message
def callback_orientation(data):
    client.update_configuration({"position_yaw_init":data.z})

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    sub_orientation = rospy.Subscriber('orientation_degrees', Vector3, callback_orientation, queue_size=1, tcp_nodelay=True)

    r = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        r.sleep()
