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
from geometry_msgs.msg import PoseWithCovarianceStamped as messagetype
from sensor_fusion_comm.msg import DoubleArrayStamped as messagetypeout

class PosErrLeica:
  def __init__(self):
    self.ground_truth_=rospy.get_param("~ground_truth","")
    self.eps_=10000000.0
    self.csvfile_=open(os.path.join(self.ground_truth_, "data.csv"), "r")
    self.truth_reader_=list(csv.reader(self.csvfile_))
    #start reading at line 1 (i.e. second line) since first is expected to be header
    self.transform_=np.array([0,0,0])
    self.linenr_=1
    self.nlines_=len(self.truth_reader_)
    self.init_meas_=False
    
    #init publisher
    self.pub_=rospy.Publisher("pos_error_leica/output", messagetypeout, queue_size=20)
    
  def l2_norm(self, arrin, truth):
    return np.linalg.norm(arrin-truth, None)


  def linf_norm(self, arrin, truth):
    return np.linalg.norm(arrin-truth, np.inf)

  def difference(self, arrin, truth):
    return arrin-truth

  def callback(self, data):
    if(self.init_meas_):
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
     
      arrin=np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])+self.transform_
      
      #error for position
      outputp1=self.l2_norm(arrin[0:3], truth[0:3])
      outputp2=self.linf_norm(arrin[0:3], truth[0:3])
      outputp3=self.difference(arrin[0:3],truth[0:3])
      
      
      #outputs: l2 norm(position), inf norm(position), l2 norm(velocity), inf norm (velocity), differences (positions), differences (velocities)
      datanew=(arrin[0], arrin[1], arrin[2], outputp1, outputp2, outputp3[0], outputp3[1], outputp3[2])
      
      #keep timestamp from previous data and just replace actual data
      dataout=messagetypeout()
      dataout.header.stamp=data.header.stamp
      dataout.data=datanew
      self.pub_.publish(dataout)

    else:
      arrin=np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
      
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

  def listener(self):
    topic="pos_error_leica/input"
    rospy.Subscriber(topic, messagetype, self.callback)
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


