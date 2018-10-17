
import numpy as np
import tensorflow as tf
import keras

"""
model=keras.models.load_model("/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/Model/pose_working.h5")
json_string = model.to_json()
file=open("/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/Model/pose_working_arch.json", "w")
file.write(json_string)
model.save_weights("/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/Model/pose_working_weights.h5")
"""
global graphg
global sessiong
graph=tf.Graph()
with graph.as_default():
	session = tf.Session()
	with session.as_default():
		with open("/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/Model/pose_working_arch.json") as arch_file:
			model = keras.models.model_from_json(arch_file.read())
			model.load_weights("/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/Model/pose_working_weights.h5")
			testpred=np.random.rand(1,99,7)
			graphg=graph
			sessiong=session
with graphg.as_default():
	with sessiong.as_default():
		#model.load_weights("/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/Model/pose_working_weights.h5") #this is the problem how to get them saved in the model
		model.predict(testpred)
