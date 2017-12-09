#!/usr/bin/env python



#This code closely follows the pytalker/pylistener node example (http://wiki.ros.org/ROSNodeTutorialPython)
#(c) Yannick Huber (huberya)



import roslib
roslib.load_manifest('msf_updates')
import rospy
import numpy as np
import csv
import os
#import sys
#sys.path.insert(0, '~/catkin_ws/src/ethzasl_msf/msf_core')
from transformation_functions import estimate_transformation, transform_point, quaternion_to_matrix
#this is for leica
#from geometry_msgs.msg import PointStamped as leicatype
#this is for vicon
from geometry_msgs.msg import TransformStamped as leicatype
from sensor_fusion_comm.msg import DoubleArrayStamped as msftype
from geometry_msgs.msg import PoseWithCovarianceStamped as roviotype
"""
$ rosmsg show geometry_msgs/PointStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z

$ rosmsg show sensor_fusion_comm/DoubleArrayStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float64[] data

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

"""

class PosErrLeica:
  def __init__(self):
    self.init_meas_=False
    self.curr_leica_truth_=np.array([0,0,0])
    #init publisher
    self.pub_=rospy.Publisher("pos_error_leica/output", msftype, queue_size=20)
    
    estimate_tf=True
    #transformation stuff
    if estimate_tf:
      self.translation_=np.array([0,0,0]) #translation x, y, z
      self.rotation_=np.zeros((3,3)) #3x3 rotation matrix
      self.ninit_points_=rospy.get_param("~ninit_points", 100)
      self.npoints_=0
      self.points_=np.zeros((3,self.ninit_points_)) #matrix to store all init points (each point has 3 coordinates)
      self.truth_points_=np.zeros((3,self.ninit_points_)) #matrix to store all truth points (each point has 3 coordinates)
    else:
	  self.translation_=rospy.get_param("~translation", [0,0,0])
	  #quaternion is w, x, y, z
	  self.rotation_=quaternion_to_matrix(rospy.get_param("~quaternion", [1,0,0,0]))
	  self.init_meas_=True

  def l2_norm(self, arrin, truth):
    return np.linalg.norm(arrin-truth, None)


  def linf_norm(self, arrin, truth):
    return np.linalg.norm(arrin-truth, np.inf)

  def difference(self, arrin, truth):
    return arrin-truth

  #for simplicyt will always assume leica measurement arrives before according msf estimate
  #makes some sense since position is used as input
  def callbackrovio(self, data):
    if(self.init_meas_):
      arrin=transform_point(np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]), self.translation_, self.rotation_)
      
      #error for position
      outputp1=self.l2_norm(arrin[0:3], self.curr_leica_truth_[0:3])
      outputp2=self.linf_norm(arrin[0:3], self.curr_leica_truth_[0:3])
      outputp3=self.difference(arrin[0:3],self.curr_leica_truth_[0:3])
      
      
      #outputs: l2 norm(position), inf norm(position), differences (positions), transformed position
      datanew=(outputp1, outputp2, outputp3[0], outputp3[1], outputp3[2], arrin[0], arrin[1], arrin[2])
      
      #keep timestamp from previous data and just replace actual data
      dataout=msftype()
      dataout.header.stamp=data.header.stamp
      dataout.data=datanew
      self.pub_.publish(dataout)

    else:
      if np.array_equal(self.curr_leica_truth_,[0.0,0.0,0.0]):
        return
      if self.npoints_<self.ninit_points_:
        self.points_[:,self.npoints_]=np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.truth_points_[:,self.npoints_]=self.curr_leica_truth_
        self.npoints_+=1
        return
      elif self.npoints_==self.ninit_points_:
        #gives best transform from points to truth_points
        [self.translation_, self.rotation_]=estimate_transformation(self.points_, self.truth_points_)
        #print(self.translation_)
        #print(self.rotation_)
        self.npoints_+=1
        self.init_meas_=True
        return      
      else:
        print("something strange happened!!")
        return

  def callbackleica(self, data):
    #this is for leica
    #self.curr_leica_truth_=np.array([data.point.x, data.point.y, data.point.z])
    #this is for vicon
    self.curr_leica_truth_=np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
    return
    
  def listener(self):
    topicdata="pos_error_leica/datainput"
    rospy.Subscriber(topicdata, roviotype, self.callbackrovio)
    topictruth="pos_error_leica/truthinput"
    rospy.Subscriber(topictruth, leicatype, self.callbackleica)
    rospy.spin()
      
  def reconfigure(self, config, level):
    return config
      
if __name__ == '__main__':
  try:
    rospy.init_node('pos_error_leica', anonymous=True)
    gs=PosErrLeica()
    gs.listener()
  except rospy.ROSInterruptException:
    pass


