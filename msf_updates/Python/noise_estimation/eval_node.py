#!/usr/bin/env python



#This code closely follows the pytalker/pylistener node example (http://wiki.ros.org/ROSNodeTutorialPython)
#(c) Yannick Huber (huberya)


import roslib
roslib.load_manifest('msf_updates')
import rospy
import numpy as np
import tensorflow as tf
import keras
import os
import time
from sensor_fusion_comm.srv import *
from sensor_fusion_comm.msg import ArrayWithKey as datatype
import threading
"""
rosmsg show sensor_fusion_comm/ArrayWithKey 
string key
float64[] data
"""

#this is an ugly workaround for ros being multithreaded and tensorflow not liking that
#idea is to have global graphs and set the correct one as active bevore predicting
#see: https://stackoverflow.com/questions/41990014/load-multiple-models-in-tensorflow
# https://github.com/keras-team/keras/issues/6462
#and for global dictionaries: https://stackoverflow.com/questions/37009767/python-make-a-dictionary-created-in-a-function-visible-to-outside-world
global graphs
if 'graphs' not in globals():
	graphs = {}
global sessions
if 'sessions' not in globals():
	sessions = {}


class EvalObject:
	def __init__(self, modelfile, modelweights, maxmemory, evalfrequency, key):
		print("starting")
		#self.model=keras.models.load_model(modelfile)
		#how to load keras models in multithreaded environment
		graph = tf.Graph()
		with graph.as_default():
			session = tf.Session()
			with session.as_default():
				with open(modelfile) as arch_file:
					self.model = keras.models.model_from_json(arch_file.read())
					self.model.load_weights(modelweights)
					#graphs[key]=tf.get_default_graph()
					graphs[key]=graph
					sessions[key]=session

		self.memory_=[]
		self.memory_.append([]) #need 3d becase of batch (use 1 however)
		self.maxmemory_ = maxmemory
		self.evalfrequency = evalfrequency
		self.counter=int(evalfrequency/2)
		self.currprediction=0
		self.key=key
		self.lock=threading.Lock()
		print("done")
		
	def evaluate(self):
		return self.currprediction
		
	def addMeas(self, data):
		if len(self.memory_[0])<self.maxmemory_:
			self.memory_[0].append(data)
		else:
			self.memory_[0].pop(0)
			self.memory_[0].append(data)
		self.counter+=1
		if self.counter>=self.evalfrequency:
			print(self.counter)
			self.counter-=self.evalfrequency
			print(self.counter)
			print(np.asarray(self.memory_).shape)
			with graphs[self.key].as_default():
				with sessions[self.key].as_default():
					self.currprediction = self.model.predict(np.asarray(self.memory_))[0,:]
					#self.currprediction = models[self.key].predict(np.asarray(self.memory_))[0,:]
			
class TFEvaluationHandler:
	def __init__(self):
		#get all parameters from ros config file
		self.i=0
		self.ObjectDictionary = {} #we need a service that allows to add objects to this dictionary with a given key
			

	#service calls
	#adding itself as a new listener
	def handle_add_listener(self, req):
		newEvalObject = EvalObject(req.tfnetworkpath, req.tfnetworkweights, req.maxmemory, req.evalfrequency, req.key)
		self.ObjectDictionary[req.key]=newEvalObject
		print ("added listener")
		return True
		
	#a listener evaluating its network
	def handle_eval_listener(self, req):
		print("evaluating")
		self.ObjectDictionary[req.key].lock.acquire()
		result=self.ObjectDictionary[req.key].evaluate()
		self.ObjectDictionary[req.key].lock.release()
		response=EvalListenerResponse()
		if result is 0:
			response.output=[-1]
		else:
			response.output=result
		return response
			
	def callback(self, data):
		self.ObjectDictionary[data.key].lock.acquire()
		self.ObjectDictionary[data.key].addMeas(data.data)
		self.ObjectDictionary[data.key].lock.release()
		
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
