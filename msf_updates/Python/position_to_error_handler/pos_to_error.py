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
from geometry_msgs.msg import PointStamped as leicatype
#this is for vicon
from geometry_msgs.msg import TransformStamped as vicontype
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

class PosToError:
  def __init__(self):
    #this parameter is set true after the transformation is computed
    #once true future measurements will be compared against truth and
    #the error is being published
    self.init_meas_=False
    
    #this saves the current truth position and is updated whenever new data 
    #from the truth source, i.e. leica or vicon is available
    self.curr_truth_=np.array([0,0,0])
    
    #init publisher
    self.pub_=rospy.Publisher("pos_to_error/output", msftype, queue_size=20)
    
    #setting to decide which truth type is used. Default is leica.
    self.truth_type_=rospy.get_param("~truth_type", "leica")
    
    #setting to decide which data type is used. Default is msf.
    self.data_type_=rospy.get_param("~data_type", "msf")
    #setting wether transformation should be estimated from data (Default)
    #or wether its given in settings
    estimate_tf=rospy.get_param("~estimate_tf", True)
    #variables to initialize transformation
    if estimate_tf:
      self.translation_=np.array([0,0,0]) #translation x, y, z
      self.rotation_=np.zeros((3,3)) #3x3 rotation matrix
      #number of points from which transformation should be estimated.
      #If these are more then 3 error is minimized in a least square manner
      self.ninit_points_=rospy.get_param("~ninit_points", 100)
      self.npoints_=0
      self.points_=np.zeros((3,self.ninit_points_)) #matrix to store all init points (each point has 3 coordinates)
      self.truth_points_=np.zeros((3,self.ninit_points_)) #matrix to store all truth points (each point has 3 coordinates)
    else:
      self.translation_=rospy.get_param("~translation", [0,0,0])#translation x, y, z
      #rotation is given as a quaternion. quaternion is w, x, y, z
      self.rotation_=quaternion_to_matrix(rospy.get_param("~quaternion", [1,0,0,0]))#3x3 rotation matrix.
      print(self.translation_)
      print(self.rotation_)
      self.init_meas_=True

  #computes the l2 norm of the difference between arrin and truth.
  #Both should be 1-D np arrays with same dimension
  #output is a double
  def l2_norm(self, arrin, truth):
    return np.linalg.norm(arrin-truth, None)

  #computes the maximum norm of the difference between arrin and truth.
  #Both should be 1-D np arrays with same dimension
  #output is a double
  def linf_norm(self, arrin, truth):
    return np.linalg.norm(arrin-truth, np.inf)

  #computes the difference between arrin and truth.
  #Both should be 1-D np arrays with same dimension
  #output is a 1-D np array with same dimension
  def difference(self, arrin, truth):
    return arrin-truth

  #for simplicyt will always assume truth measurement arrives before according msf estimate
  #but since frequency is relatively a high and velocity low error caused by this is small
  def callbackmsf(self, data):
    #if transformation is available
    if(self.init_meas_):
      #transforms the input to truth frame with given transformation
      #print("n")
      #print(np.array([data.data[0], data.data[1], data.data[2]]))
      arrin=transform_point(np.array([data.data[0], data.data[1], data.data[2]]), self.translation_, self.rotation_)
      #print(arrin)
      #compute error for input
      outputp1=self.l2_norm(arrin[0:3], self.curr_truth_[0:3])
      outputp2=self.linf_norm(arrin[0:3], self.curr_truth_[0:3])
      outputp3=self.difference(arrin[0:3],self.curr_truth_[0:3])
      
      
      #outputs: l2 norm(position), inf norm(position), differences (positions), transformed position
      datanew=(outputp1, outputp2, outputp3[0], outputp3[1], outputp3[2], arrin[0], arrin[1], arrin[2])
      
      #keep timestamp from previous data and just replace actual data
      #then publish
      dataout=msftype()
      dataout.header.stamp=data.header.stamp
      dataout.data=datanew
      self.pub_.publish(dataout)
    
    #transformation not available. Need to estimate
    else:
      #make sure measurements started
      if np.array_equal(self.curr_truth_,[0.0,0.0,0.0]):
        return
      #assemble points until ninit_points are collected
      if self.npoints_<self.ninit_points_:
        self.points_[:,self.npoints_]=np.array([data.data[0], data.data[1], data.data[2]])
        self.truth_points_[:,self.npoints_]=self.curr_truth_
        self.npoints_+=1
        return
      #compute best transform with given points (in least square manner)
      elif self.npoints_==self.ninit_points_:
        [self.translation_, self.rotation_]=estimate_transformation(self.points_, self.truth_points_)
        self.npoints_+=1
        self.init_meas_=True
        print(self.translation_)
        print(self.rotation_)
        return   
      else:
        print("something strange happened!!")
        return
  
  #for simplicity will always assume truth measurement arrives before according msf estimate
  #but since frequency is relatively a high and velocity low error caused by this is small
  def callbackrovio(self, data):
    #if transformation is available
    if(self.init_meas_):
      #transforms the input to truth frame with given transformation
      arrin=transform_point(np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]), self.translation_, self.rotation_)
      
      #compute error for input
      outputp1=self.l2_norm(arrin[0:3], self.curr_truth_[0:3])
      outputp2=self.linf_norm(arrin[0:3], self.curr_truth_[0:3])
      outputp3=self.difference(arrin[0:3],self.curr_truth_[0:3])
      
      
      #outputs: l2 norm(position), inf norm(position), differences (positions), transformed position
      datanew=(outputp1, outputp2, outputp3[0], outputp3[1], outputp3[2], arrin[0], arrin[1], arrin[2])
      
      #keep timestamp from previous data and just replace actual data
      #then publish
      dataout=msftype()
      dataout.header.stamp=data.header.stamp
      dataout.data=datanew
      self.pub_.publish(dataout)
    
    #transformation not available. Need to estimate
    else:
      #make sure measurements started
      if np.array_equal(self.curr_truth_,[0.0,0.0,0.0]):
        return
      #assemble points until ninit_points are collected
      if self.npoints_<self.ninit_points_:
        self.points_[:,self.npoints_]=np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.truth_points_[:,self.npoints_]=self.curr_truth_
        self.npoints_+=1
        return
      #compute best transform with given points (in least square manner)
      elif self.npoints_==self.ninit_points_:
        #print(self.points_)
        #print(self.truth_points_)
        [self.translation_, self.rotation_]=estimate_transformation(self.points_, self.truth_points_)
        self.npoints_+=1
        self.init_meas_=True
        print(self.translation_)
        print(self.rotation_)
        return   
      else:
        print("something strange happened!!")
        return
        
  def callbackleica(self, data):
    #this is for leica
    self.curr_truth_=np.array([data.point.x, data.point.y, data.point.z])
    return
    
  def callbackvicon(self, data):
    #print("vicon cb")
    #this is for vicon
    self.curr_truth_=np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
    return
    
  def listener(self):
    topicdata="pos_to_error/datainput"
    #case switch for data input (either rovio or msf)
    if self.data_type_=="rovio":
      rospy.Subscriber(topicdata, roviotype, self.callbackrovio)
    elif self.data_type_=="msf":
      rospy.Subscriber(topicdata, msftype, self.callbackmsf)
    else:
	    print("Not supported data type: "+str(self.data_type_))
    topictruth="pos_to_error/truthinput"
    #case switch for truth input since datatype is differently
    if self.truth_type_=="leica":
        rospy.Subscriber(topictruth, leicatype, self.callbackleica)
    elif self.truth_type_=="vicon":
	    rospy.Subscriber(topictruth, vicontype, self.callbackvicon)
    else:
	    print("Not supported groundtruth type: "+str(self.truth_type_))
    rospy.spin()
  
  #template for dynamic reconfigure server (currently unused)    
  def reconfigure(self, config, level):
    return config
      
if __name__ == '__main__':
  try:
    rospy.init_node('pos_to_error', anonymous=True)
    gs=PosToError()
    gs.listener()
  except rospy.ROSInterruptException:
    pass


