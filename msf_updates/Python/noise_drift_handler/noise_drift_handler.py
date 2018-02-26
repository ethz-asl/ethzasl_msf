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
#import sys
#sys.path.insert(0, '~/catkin_ws/src/ethzasl_msf/msf_core
#need pose_with_covariance (as input for msf) and position if want to apply noise there as well
from geometry_msgs.msg import PoseWithCovarianceStamped as posetype
from geometry_msgs.msg import PointStamped as positiontype
from geometry_msgs.msg import TransformStamped as transformtype
from sensor_msgs.msg import NavSatFix as gpstype
from geometry_msgs.msg import Vector3Stamped as eventtype
"""
$ rosmsg show geometry_msgs/PoseWithCovarianceStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance

$ rosmsg show geometry_msgs/PointStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z
  
$ rosmsg show geometry_msgs/TransformStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/Transform transform
  geometry_msgs/Vector3 translation
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation
    float64 x
    float64 y
    float64 z
    float64 w

$ rosmsg show sensor_msgs/NavSatFix 
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
sensor_msgs/NavSatStatus status
  int8 STATUS_NO_FIX=-1
  int8 STATUS_FIX=0
  int8 STATUS_SBAS_FIX=1
  int8 STATUS_GBAS_FIX=2
  uint16 SERVICE_GPS=1
  uint16 SERVICE_GLONASS=2
  uint16 SERVICE_COMPASS=4
  uint16 SERVICE_GALILEO=8
  int8 status
  uint16 service
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type


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

class MsfNoiseHandler:
  def __init__(self):
    #params for noise. To be set in yaml file
    #if mean is 0 then adding pure noise
    #if mean != 0 then is creating some drift aswell (not yet)
    
    #params for pose
    self.pose_mu_=rospy.get_param("~pose_noise_mean",0.0)
    self.pose_stddeviation_=rospy.get_param("~pose_noise_number_stddeviations", 0.0)
    self.pose_use_noise_=rospy.get_param("~pose_use_noise", False)
        
    self.pose_p_outlier_=rospy.get_param("~pose_probability_outlier", 0.0)
    self.pose_create_outlier_=rospy.get_param("~pose_create_outlier", False)
    self.pose_group_size_=rospy.get_param("~pose_group_size", 1) #not working rn
    self.pose_curr_group_=0
    
    #params for position
    self.position_mu_=rospy.get_param("~position_noise_mean",0.0)
    self.position_stddeviation_=rospy.get_param("~position_noise_number_stddeviations", 0.0)
    self.position_use_noise_=rospy.get_param("~position_use_noise", False)
        
    self.position_p_outlier_=rospy.get_param("~position_probability_outlier", 0.0)
    self.position_create_outlier_=rospy.get_param("~position_create_outlier", False)
    self.position_group_size_=rospy.get_param("~position_group_size", 1) #not working rn
    self.position_curr_group_=0

    
    
    #params for transform (we handle NavSatFix as transform for params)
    self.transform_mu_=rospy.get_param("~transform_noise_mean",0.0)
    self.transform_stddeviation_=rospy.get_param("~transform_noise_number_stddeviations", 0.0)
    self.transform_use_noise_=rospy.get_param("~transform_use_noise", False)
        
    self.transform_p_outlier_=rospy.get_param("~transform_probability_outlier", 0.0)
    self.transform_create_outlier_=rospy.get_param("~transform_create_outlier", False)
    self.transform_group_size_=rospy.get_param("~transform_group_size", 1) #not working rn
    self.transform_curr_group_=0
    self.transform_use_fixed_diverge_time_=rospy.get_param("~transform_diverge_fixed_time", False)
    self.transform_diverge_frame_=rospy.get_param("~transform_diverge_frame", 0)
    self.transform_diverge_length_=rospy.get_param("~transform_diverge_length", 0)
    self.transform_curr_frame_=0
    
    #params to estimate stddeviation of data 
    #to be set manually
    self.ninit_=350
    self.started_=False
    self.nrecv_=0
    #self.stddeviation_=0
    #self.datamean_=0
    
    #init publisher
    self.pose_pub_=rospy.Publisher("noise_drift_handler/pose_output", posetype, queue_size=20)
    self.position_pub_=rospy.Publisher("noise_drift_handler/position_output", positiontype, queue_size=20)
    self.gps_pub_ = rospy.Publisher("noise_drift_handler/gps_output", gpstype, queue_size=20)
    
    #this will publish (3,0,0) if rovio starts diverging and (4,0,0) if it stops diverging
    self.events_pub_ = rospy.Publisher("noise_drift_handler/events", eventtype, queue_size=20)
  
  #adds gaussian distributed noise with mu and stddeviation   
  def add_noise(self, arrin, mu, stddeviation):
    noise=np.random.normal(mu, stddeviation, len(arrin))
    arrout=arrin+noise
    return arrout

  #creates an oultier based on arring in by adding or subtracting 100 times stddeviation to arrin
  def create_outlier(self, arrin, stddeviation):
    #add some big disturbance to arrin (for now 100 times stddeviation)
    #sign=np.random.randint(0,2) #need 0/1 random
    #noise=np.array([100*stddeviation for i in range(len(arrin))])
    noise = np.random.normal(0, 100*stddeviation, len(arrin))
    #if sign:
    #  arrout=arrin+noise
    #else:
    #  arrout=arrin-noise
    arrout=arrin+noise
    return arrout

  #callback for messages. First decides which of the supported data types (currently pose_with_covariance_stamped
  #and PointStamped) is current input and then manipulates them according to params before outputting them in their
  #corresponding chanel
  def callback(self, data):
    #decide which datatype is current and set params accordingly
    dtype=str(data._type)
    if dtype=="geometry_msgs/PoseWithCovarianceStamped":
      dataarr=np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
      data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
      data.pose.pose.orientation.w])
      
      stddeviation=self.pose_stddeviation_
      use_noise=self.pose_use_noise_
      mu=self.pose_mu_
      
      p_outlier=self.pose_p_outlier_
      create_outlier=self.pose_create_outlier_
      group_size=self.pose_group_size_
      curr_group=self.pose_curr_group_
    elif dtype=="geometry_msgs/PointStamped":
      dataarr=np.array([data.point.x, data.point.y, data.point.z])
      
      stddeviation=self.position_stddeviation_
      use_noise=self.position_use_noise_
      mu=self.position_mu_
      
      p_outlier=self.position_p_outlier_
      create_outlier=self.position_create_outlier_
      group_size=self.position_group_size_
      curr_group=self.position_curr_group_
    elif dtype=="geometry_msgs/TransformStamped":
      dataarr=np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
      self.transform_curr_frame_+=1
      if self.transform_use_fixed_diverge_time_ and self.transform_curr_frame_==self.transform_diverge_frame_:
        self.transform_curr_group_=self.transform_diverge_length_
        eventout = eventtype()
        eventout.header = data.header
        eventout.vector.x = 3
        eventout.vector.y = 0
        eventout.vector.z = 0
        self.events_pub_.publish(eventout)
        print("diverging gps")
      stddeviation=self.transform_stddeviation_
      use_noise=self.transform_use_noise_
      mu=self.transform_mu_
      
      p_outlier=self.transform_p_outlier_
      create_outlier=self.transform_create_outlier_
      group_size=self.transform_group_size_
      curr_group=self.transform_curr_group_
    elif dtype=="sensor_msgs/NavSatFix":
      dataarr=np.array([data.latitude, data.longitude, data.altitude])
      self.transform_curr_frame_+=1
      if self.transform_use_fixed_diverge_time_ and self.transform_curr_frame_==self.transform_diverge_frame_:
        self.transform_curr_group_=self.transform_diverge_length_
        eventout = eventtype()
        eventout.header = data.header
        eventout.vector.x = 3
        eventout.vector.y = 0
        eventout.vector.z = 0
        self.events_pub_.publish(eventout)
        print("diverging gps")
      stddeviation=self.transform_stddeviation_
      use_noise=self.transform_use_noise_
      mu=self.transform_mu_
      
      p_outlier=self.transform_p_outlier_
      create_outlier=self.transform_create_outlier_
      group_size=self.transform_group_size_
      curr_group=self.transform_curr_group_
    else:
      print("not supported datatype:")
      print(dtype)

    if(self.started_):
      #change data according to params choosen above
      if use_noise:
        dataarr=self.add_noise(dataarr, mu, stddeviation)
      if create_outlier:
        t=np.random.uniform()
        if curr_group>0:
          #print("creating grouped outlier")
          dataarr=self.create_outlier(dataarr, stddeviation)
          if dtype=="geometry_msgs/PoseWithCovarianceStamped":
            self.pose_curr_group_-=1
          elif dtype=="geometry_msgs/PointStamped":
            self.position_curr_group_-=1
          elif dtype=="geometry_msgs/TransformStamped":
            self.transform_curr_group_-=1
            if self.transform_curr_group_==0:
              eventout = eventtype()
              eventout.header = data.header
              eventout.vector.x = 4
              eventout.vector.y = 0
              eventout.vector.z = 0
              self.events_pub_.publish(eventout)
          elif dtype=="sensor_msgs/NavSatFix":
            self.transform_curr_group_-=1
            if self.transform_curr_group_==0:
              eventout = eventtype()
              eventout.header = data.header
              eventout.vector.x = 4
              eventout.vector.y = 0
              eventout.vector.z = 0
              self.events_pub_.publish(eventout)
        elif t<p_outlier:
          dataarr=self.create_outlier(dataarr, stddeviation)
          if dtype=="geometry_msgs/PoseWithCovarianceStamped":
            self.pose_curr_group_=self.pose_group_size_-1
          elif dtype=="geometry_msgs/PointStamped":
            self.position_curr_group=self.position_group_size_-1
          elif dtype=="geometry_msgs/TransformStamped":
            self.transform_curr_group=self.transform_group_size_-1
          elif dtype=="geometry_msgs/NavSatFix":
            self.transform_curr_group=self.transform_group_size_-1
    else:
      self.nrecv_+=1
      if self.nrecv_>=self.ninit_:
        print("starting noise")
        self.started_=True 
        
    #print(dataarr[0])
    if dtype=="geometry_msgs/PoseWithCovarianceStamped":
      data.pose.pose.position.x = dataarr[0]
      data.pose.pose.position.y = dataarr[1]
      data.pose.pose.position.z = dataarr[2]
      data.pose.pose.orientation.x = dataarr[3]
      data.pose.pose.orientation.y = dataarr[4]
      data.pose.pose.orientation.z = dataarr[5]
      data.pose.pose.orientation.w = dataarr[6]
      #print("second "+str(data.pose.pose.position.x))
      self.pose_pub_.publish(data)
    elif dtype=="geometry_msgs/PointStamped":
      data.point.x = dataarr[0]
      data.point.y = dataarr[1]
      data.point.z = dataarr[2]
      self.position_pub_.publish(data)
    elif dtype=="geometry_msgs/TransformStamped":
      datan=positiontype()
      datan.header = data.header
      datan.point.x = dataarr[0]
      datan.point.y = dataarr[1]
      datan.point.z = dataarr[2]
      self.position_pub_.publish(datan)
    elif dtype=="sensor_msgs/NavSatFix":
      datan=gpstype()
      datan.header = data.header
      datan.status = data.status
      datan.latitude = dataarr[0]
      datan.longitude = dataarr[1]
      datan.altitude = dataarr[2]
      datan.position_covariance = data.position_covariance
      datan.position_covariance_type = data.position_covariance_type
      self.gps_pub_.publish(datan)
    else:
      print("not supported datatype (output):")
      print(dtype)
    
  def listener(self):
    topic_pose="noise_drift_handler/pose_input"
    rospy.Subscriber(topic_pose, posetype, self.callback)
    topic_position="noise_drift_handler/position_input"
    rospy.Subscriber(topic_position, positiontype, self.callback)
    topic_transform="noise_drift_handler/transform_input"
    rospy.Subscriber(topic_transform, transformtype, self.callback)
    topic_gps="noise_drift_handler/gps_input"
    rospy.Subscriber(topic_gps, gpstype, self.callback)
    rospy.spin()
      
  def reconfigure(self, config, level):
    return config
      
if __name__ == '__main__':
  try:
    rospy.init_node('noise_drift_handler', anonymous=True)
    gs=MsfNoiseHandler()
    gs.listener()
  except rospy.ROSInterruptException:
    pass
