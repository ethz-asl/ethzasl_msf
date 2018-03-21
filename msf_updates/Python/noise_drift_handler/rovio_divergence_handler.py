#!/usr/bin/env python



#This code closely follows the pytalker/pylistener node example (http://wiki.ros.org/ROSNodeTutorialPython)
#(c) Yannick Huber (huberya)


import roslib
roslib.load_manifest('msf_updates')
import rospy
import numpy as np
import csv
import os
from math import sqrt

from sensor_msgs.msg import Image as imagetype
from sensor_msgs.msg import Imu as imutype
from geometry_msgs.msg import Vector3Stamped as eventtype

"""
$ rosmsg show sensor_msgs/Image 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance

rosmsg show geometry_msgs/Vector3Stamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Vector3 vector
  float64 x
  float64 y
  float64 z

"""

class RovioDivergenceHandler:
  def __init__(self):
    #params for divergence. To be set in yaml file
    #either can choose to set probability do diverge or fixed time to diverge(might use this for plot)
    
    #params for pose
    self.use_fixed_time_=rospy.get_param("~use_fixed_time", False)
    self.start_frame_=rospy.get_param("~divergence_start_frame", 1000)
    self.group_size_=rospy.get_param("~divergence_group_size", 1)
    self.prob_divergence_=rospy.get_param("~probability_divergence", 0.0)
    print("active")
    print(self.start_frame_)
    
    self.curr_group_size_ = 0
    self.curr_frame_ = 0
    
    #init publisher
    self.cam0_pub_=rospy.Publisher("rovio_divergence_handler/cam0_output", imagetype, queue_size=20)
    self.cam1_pub_=rospy.Publisher("rovio_divergence_handler/cam1_output", imagetype, queue_size=20)
    self.imu0_pub_=rospy.Publisher("rovio_divergence_handler/imu0_output", imutype, queue_size=20)
    
    #this will publish (1,0,0) if rovio starts diverging and (2,0,0) if it stops diverging
    self.events_pub_ = rospy.Publisher("rovio_divergence_handler/events", eventtype, queue_size=20)
  

  #makes rovio diverge by setting all data to 0
  def create_divergence(self, data):
    length = len(data)
    data = [255]*length
    #print(type(data))
    return data

  #makes rovio diverge by corrupting Imu measurement
  def create_divergence_imu(self, data):
    return 5*data

  #callback for cam0
  def callback0(self, data):
    self.curr_frame_+=1
    #print(type(data.data))
    if self.curr_group_size_==0:
      #print("done diverging")
      if self.use_fixed_time_:
        if self.curr_frame_==self.start_frame_:
          self.curr_group_size_ = self.group_size_
          eventout = eventtype()
          eventout.header = data.header
          eventout.vector.x = 1
          eventout.vector.y = 0
          eventout.vector.z = 0
          self.events_pub_.publish(eventout)
          print("diverging")
      else:
        p = np.random.uniform()
        if p < self.prob_divergence_:
          self.curr_group_size_ = self.group_size_
          self.prob_divergence_=0.0
          print("diverging")
    else:
      self.curr_group_size_-=1
      if self.curr_group_size_==0:
        eventout = eventtype()
        eventout.header = data.header
        eventout.vector.x = 2
        eventout.vector.y = 0
        eventout.vector.z = 0
        self.events_pub_.publish(eventout)
      data.data = self.create_divergence(data.data)
    self.cam0_pub_.publish(data)
    
  #callback for cam1
  #decision is made in cam0 callback: this just applies if group>0 for cam1
  def callback1(self, data):
    if self.curr_group_size_>0:
      data.data = self.create_divergence(data.data)
    self.cam1_pub_.publish(data)
    
  def callbackimu(self,data):
    if self.curr_group_size_>0:
      arrin = np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
      arrout = self.create_divergence_imu(arrin)
      data.angular_velocity.x=arrout[0]
      data.angular_velocity.y=arrout[1]
      data.angular_velocity.z=arrout[2]
    self.imu0_pub_.publish(data)
  def listener(self):
    topic_cam0="rovio_divergence_handler/cam0_input"
    rospy.Subscriber(topic_cam0, imagetype, self.callback0)
    topic_cam1="rovio_divergence_handler/cam1_input"
    rospy.Subscriber(topic_cam1, imagetype, self.callback1)
    topic_imu0="rovio_divergence_handler/imu0_input"
    rospy.Subscriber(topic_imu0, imutype, self.callbackimu)
    rospy.spin()
      
  def reconfigure(self, config, level):
    return config
      
if __name__ == '__main__':
  try:
    rospy.init_node('rovio_divergence_handler', anonymous=True)
    gs=RovioDivergenceHandler()
    gs.listener()
  except rospy.ROSInterruptException:
    pass
