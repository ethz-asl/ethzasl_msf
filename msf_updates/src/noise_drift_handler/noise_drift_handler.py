#This code closely follows the pytalker/pylistener node example (http://wiki.ros.org/ROSNodeTutorialPython)
#(c) Yannick Huber (huberya)


#!/usr/bin/env python

import roslib
roslib.load_manifest('MsfNoiseHandler')
import rospy
import numpy as np

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

# Import custom message data and dynamic reconfigure variables. think about this
from MsfNoiseHandler.msg import node_example_data
from MsfNoiseHandler.yaml import msf_noise_handler_params as ConfigType

class MsfNoiseHandler:
  def __init__(self):
    #params for noise. To be set in yaml file
    #if mean is 0 then adding pure noise
    #if mean != 0 then is creating some drift aswell
    self.mu=rospy.get_param("~noise_mean",0.0)
    self.sigma=rospy.get_param("~noise_variance", 0.0)
    self.use_noise=rospy.get_param("~use_noise", False)
		
    #params for outlier creation. To be set in yaml file
    self.p_outlier=rospy.get_param("~probability_outlier", 0.0)
    self.use_outlier=rospy.get_param("~create_outlier", False)
    self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
	
  def add_noise(self, arrin):
    noise=np.random.normal(self.mu, self.sigma, len(arrin))
    arrout=arrin+noise
    return arrout


  def create_outlier(self, arrin):
    #add some big disturbance to arrin
    return arrout

  def odometry_callback(self, data):
    #handle data here and add noise if necessary


if __name__ == '__main__':
  try:
    rospy.init_node('noise_drift_handler', anonymous=True)
    gs=MsfNoiseHandler()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
