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
"""

class PosErrLeica:
  def __init__(self):
    #not comparing with csv file but rosbag instead: dont need these params
    #self.ground_truth_=rospy.get_param("~ground_truth","")
    #self.eps_=10000000.0
    #self.csvfile_=open(os.path.join(self.ground_truth_, "data.csv"), "r")
    #self.truth_reader_=list(csv.reader(self.csvfile_))
    #start reading at line 1 (i.e. second line) since first is expected to be header
    

    #self.linenr_=1
    #self.nlines_=len(self.truth_reader_)
    self.init_meas_=False
    self.curr_leica_truth_=np.array([0,0,0])
    #init publisher
    self.pub_=rospy.Publisher("pos_error_leica/output", msftype, queue_size=20)
    self.truth_type_=rospy.get_param("~truth_type", "leica")
    estimate_tf=rospy.get_param("~estimate_tf", True)
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
	  print(self.translation_)
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
  def callbackmsf(self, data):
    if(self.init_meas_):
      #dont need csv part anymore
      """
      #print("callback")
      timestamp=float(str(data.header.stamp))
      #print (timestamp)
      truth=0
      while self.linenr_<self.nlines_:
        if(timestamp-self.eps_>float(self.truth_reader_[self.linenr_][0])):
          self.linenr_+=1
        elif(timestamp+self.eps_>float(self.truth_reader_[self.linenr_][0])):
          truth=np.array([float(self.truth_reader_[self.linenr_][1]), float(self.truth_reader_[self.linenr_][2]),
          float(self.truth_reader_[self.linenr_][3])])
          break

        else:
          return
      """
      #transforms the point input to leica frame by best transformation computed on init
      arrin=transform_point(np.array([data.data[0], data.data[1], data.data[2]]), self.translation_, self.rotation_)
      
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
      #computing transform from points. I'm pretty sure there is such a function existing, but I'm too 
      #stupid to find it
      if np.array_equal(self.curr_leica_truth_,[0.0,0.0,0.0]):
        return
      if self.npoints_<self.ninit_points_:
        self.points_[:,self.npoints_]=np.array([data.data[0], data.data[1], data.data[2]])
        self.truth_points_[:,self.npoints_]=self.curr_leica_truth_
        self.npoints_+=1
        return
      elif self.npoints_==self.ninit_points_:
        #gives best transform from points to truth_points
        [self.translation_, self.rotation_]=estimate_transformation(self.points_, self.truth_points_)
        self.npoints_+=1
        self.init_meas_=True
        return      
      else:
        print("something strange happened!!")
        return
      #dont need csv part anymore
      """
      timestamp=float(str(data.header.stamp))
      #print(timestamp)
      truth=0
      while self.linenr_<self.nlines_:
        #print(self.linenr_)
        #print("new iter")
        #print(float(self.truth_reader_[self.linenr_][0]))
        #print(timestamp)
        if(timestamp-self.eps_>float(self.truth_reader_[self.linenr_][0])):
          
          self.linenr_+=1
        elif(timestamp+self.eps_>float(self.truth_reader_[self.linenr_][0])):
          truth=np.array([float(self.truth_reader_[self.linenr_][1]), float(self.truth_reader_[self.linenr_][2]),
          float(self.truth_reader_[self.linenr_][3])])
          self.init_meas_=True
          print("initializing")
          self.transform_=truth-arrin
          break
        else:
          print("no measurement to initialize found")
          #print(float(self.truth_reader_[self.linenr_][0]))
          #print(timestamp)
          #self.linenr_=1
          return
      """
  def callbackleica(self, data):
    #this is for leica
    self.curr_leica_truth_=np.array([data.point.x, data.point.y, data.point.z])
    return
  def callbackvicon(self, data):
	#this is for vicon
    self.curr_leica_truth_=np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
    return
  def listener(self):
    topicdata="pos_error_leica/datainput"
    rospy.Subscriber(topicdata, msftype, self.callbackmsf)
    topictruth="pos_error_leica/truthinput"
    if self.truth_type_=="leica":
        rospy.Subscriber(topictruth, leicatype, self.callbackleica)
    elif self.truth_type_=="vicon":
	    rospy.Subscriber(topictruth, vicontype, self.callbackvicon)
    else:
	    print("Not supported groundtruth type: "+str(self.truth_type))
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


