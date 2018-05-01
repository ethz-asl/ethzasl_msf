#!/usr/bin/env python



#This code closely follows the pytalker/pylistener node example (http://wiki.ros.org/ROSNodeTutorialPython)
#(c) Yannick Huber (huberya)


import roslib
roslib.load_manifest('msf_updates')
import rospy
import numpy as np
import tensorflow as tf
import os
from sensor_fusion_comm.srv import *
from sensor_fusion_comm.msg import ArrayWithKey as datatype
"""
rosmsg show sensor_fusion_comm/ArrayWithKey 
string key
float64[] data
"""
def load_graph(path_to_model):
	"""Creates a graph from saved GraphDef file and returns a saver."""
	# Creates graph from saved graph_def.pb.
	model_filename = path_to_model
	with tf.gfile.GFile(model_filename, 'rb') as f:
		graph_def = tf.GraphDef()
		graph_def.ParseFromString(f.read())

	with tf.Graph().as_default() as graph:
		tf.import_graph_def(
			graph_def, 
			input_map=None, 
			return_elements=None, 
			name="", 
			op_dict=None, 
			producer_op_list=None
		)
	return graph



class EvalObject:
	#tfnetworkpath is the path to the tensorflow model (*.pb) to be evaluated
	#frequency gives the number of data after which the model should be evaluated
	#maxsequencelength gives the maximal memory length
	#inputname is the name of the input tensor and outputname 
	#the name of the outputtensor
	def __init__(self, tfnetworkpath, maxmemory, inputname="inputs", outputname="outputs"):
		self.tfnetworkpath_=tfnetworkpath
		self.tfnetwork_ = load_graph(self.tfnetworkpath_) #the actual tfnetwork
		self.memory_=[]
		self.memory_.append([]) #need 3d becase of batch (use 1 however)
		self.maxmemory_ = maxmemory
		self.inputname_=inputname
		self.outputname_=outputname
	
	def evaluate(self):
		with tf.Session(graph=self.tfnetwork_) as sess:
			input_tensor = sess.graph.get_tensor_by_name(self.inputname_)
			tempmemory=np.einsum('ijk->jik', np.asarray(self.memory_))
			print(tempmemory.shape)
			predictions_tensor = sess.graph.get_tensor_by_name(self.outputname_)
			predictions = sess.run(predictions_tensor,
                           {input_tensor: tempmemory})#this specifies the input layer I think

			return predictions[-1,0,:]
	def addMeas(self, data):
		if len(self.memory_)<self.maxmemory_:
			self.memory_[0].append(data)
		else:
			self.memory_[0].pop(0)
			self.memory_[0].append(data)

class TFEvaluationHandler:
	def __init__(self):
		#get all parameters from ros config file
		
		self.ObjectDictionary = {} #we need a service that allows to add objects to this dictionary with a given key
			

	#service calls
	#adding itself as a new listener
	def handle_add_listener(self, req):
		newEvalObject = EvalObject(req.tfnetworkpath, req.maxmemory, req.inputname, req.outputname)
		self.ObjectDictionary[req.key]=newEvalObject
		print ("added listener")
		return True
		
	#a listener evaluating its network
	def handle_eval_listener(self, req):
		print("evaluating")
		result=self.ObjectDictionary[req.key].evaluate() 
		response=EvalListenerResponse()
		response.output=result
		return response
			
	def callback(self, data):
		self.ObjectDictionary[data.key].addMeas(data.data)
		
	#creating the service calse
	def add_listener_server(self):
		s1 = rospy.Service('eval_node/add_listener', AddListener, self.handle_add_listener)
		s2 = rospy.Service('eval_node/eval_listener', EvalListener, self.handle_eval_listener)
		print("service call ready")
		topic_input="eval_node/data_input"
		rospy.Subscriber(topic_input, datatype, self.callback)
		rospy.spin()
      
	def reconfigure(self, config, level):
		return config
      
if __name__ == '__main__':
	try:
		rospy.init_node('tf_evaluation_handler', anonymous=True)
		gs=TFEvaluationHandler()
		gs.add_listener_server()
	except rospy.ROSInterruptException:
		pass
