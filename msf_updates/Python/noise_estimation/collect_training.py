#!/bin/python

import numpy as np
import random
import os
from time import sleep
import subprocess

LAUNCH_FILE="create_ts_VH.launch"
ROSBAG_FILE="/home/yannick/Downloads/V1_01_easy.bag"
CONFIG_PATH="/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/params/create_ts_VH"
BASE_POSE_DEV_P=0.05
BASE_POSE_DEV_Q=0.02
POSITION_FEATURE_FILE="/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/data/train_position_feats.txt"
POSITION_LABEL_FILE="/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/data/train_position_labels.txt"
POSE_FEATURE_FILE="/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/data/train_pose_feats.txt"
POSE_LABEL_FILE="/home/yannick/catkin_ws/src/ethzasl_msf/msf_updates/Python/noise_estimation/data/train_pose_labels.txt"
N_RUNS = 150
P_MIN_NOISE = 0.05
P_MAX_NOISE = 2.0
Q_MIN_NOISE = 0.0005
Q_MAX_NOISE = 0.05
MIN_P_OUTLIER = 0.0
MAX_P_OUTLIER = 0.3
NOISE_TYPE = ["normal"]#, "..." #only allow symmetric noise with zero mean
META_NOISE_MEAN_MAX = 0.01
META_NOISE_DEV_MAX = 0.1 
POSE_LSTM_SEQUENCE_LENGTH=20 #corresponds to the frequency of the sensor
POSITION_LSTM_SEQUENCE_LENGTH=100 #corresponds to the frequency of the sensor
#need more params such as meta noise and change of noise in general
def run_single(noise_type, position_dev, position_meta_noise_mean, position_meta_noise_dev, position_p_outlier,
		pose_dev_p, pose_dev_q, pose_meta_noise_mean_p, pose_meta_noise_dev_p,
		pose_meta_noise_mean_q, pose_meta_noise_dev_q, pose_p_outlier, launch_file, rosbag_file):
	#write configuration file for noise_drift_handler (noise_drift_handler_fix.yaml)
	with open(os.path.join(CONFIG_PATH, "noise_drift_handler_fix.yaml"), "w") as noise_drift_config:
		noise_drift_config.write(
"""
#global param
/noise_drift_handler/noise_type: """+str(noise_type)+"""
#params for pose
/noise_drift_handler/pose_noise_mean: 0.0
/noise_drift_handler/pose_noise_stddeviation_p: """+str(pose_dev_p)+"""
/noise_drift_handler/pose_noise_stddeviation_q: """+str(pose_dev_q)+"""
/noise_drift_handler/pose_use_noise: true
/noise_drift_handler/pose_probability_outlier: """+str(pose_p_outlier)+"""
/noise_drift_handler/pose_group_size: 1
/noise_drift_handler/pose_create_outlier: true
/noise_drift_handler/pose_create_ts: true
#need to estimate this two apriori with some smart data analytics TODO
/noise_drift_handler/pose_original_noise_p: """+str(BASE_POSE_DEV_P)+"""
/noise_drift_handler/pose_original_noise_q: """+str(BASE_POSE_DEV_Q)+"""
/noise_drift_handler/pose_path_to_ts: """+str(POSE_LABEL_FILE)+"""
#params for meta noise
/noise_drift_handler/pose_lstm_sequence_length: """+str(POSE_LSTM_SEQUENCE_LENGTH)+"""
/noise_drift_handler/pose_use_meta_noise: True
/noise_drift_handler/pose_meta_noise_mean_p: """+str(pose_meta_noise_mean_p)+"""
/noise_drift_handler/pose_meta_noise_dev_p: """+str(pose_meta_noise_dev_p)+"""
/noise_drift_handler/pose_meta_noise_mean_q: """+str(pose_meta_noise_mean_q)+"""
/noise_drift_handler/pose_meta_noise_dev_q: """+str(pose_meta_noise_dev_q)+"""

#params for transform
/noise_drift_handler/transform_noise_mean: 0.0
/noise_drift_handler/transform_noise_stddeviation_p: """+str(position_dev)+"""
/noise_drift_handler/transform_use_noise: true
/noise_drift_handler/transform_probability_outlier: """+str(position_p_outlier)+"""
/noise_drift_handler/transform_group_size: 1
/noise_drift_handler/transform_create_outlier: true
/noise_drift_handler/transform_diverge_fixed_time: true
#/noise_drift_handler/transform_diverge_frame: 3000
/noise_drift_handler/transform_diverge_frame: 7000
/noise_drift_handler/transform_diverge_length: 1000
/noise_drift_handler/transform_create_ts: true

/noise_drift_handler/transform_original_noise_p: 0.0
/noise_drift_handler/transform_path_to_ts: """+str(POSITION_LABEL_FILE)+"""
#params for meta noise
/noise_drift_handler/position_lstm_sequence_length: """+str(POSITION_LSTM_SEQUENCE_LENGTH)+"""
/noise_drift_handler/position_use_meta_noise: True
/noise_drift_handler/position_meta_noise_mean_p: """+str(position_meta_noise_mean)+"""
/noise_drift_handler/position_meta_noise_dev_p: """+str(position_meta_noise_dev))

	#write configuration file for rovio_divergence_handler (rovio_divergence_handler_fix.yaml)
	with open(os.path.join(CONFIG_PATH, "rovio_divergence_handler_fix.yaml"), "w") as divergence_config:
		divergence_config.write(
"""#params for rovio_divergence_handler
/rovio_divergence_handler/use_fixed_time: True
#/rovio_divergence_handler/divergence_start_frame: 1600
/rovio_divergence_handler/divergence_start_frame: 800
/rovio_divergence_handler/divergence_group_size: 20
/rovio_divergence_handler/probability_divergence: 0.0""")

	
	#write configuration file for msf (only need to change initial noise to be correct) (msf_sensor_fix.yaml)
	with open(os.path.join(CONFIG_PATH, "msf_sensor_fix.yaml"), "w") as msf_config:
		#write file
		msf_config.write(
"""scale_init: 1 \n
core/data_playback: true
core/fixed_bias: false

#########IMU PARAMETERS#######
core/core_noise_acc: 0.083
core/core_noise_accbias: 0.00005
core/core_noise_gyr: 0.0013
core/core_noise_gyrbias: 0.0000013

#stablility param for sensormanager
position_pose_sensor/use_stable_initialization: false

#these params are defined in PositionPoseSensor.cfg and therefore have prefix position_pose_sensor
position_pose_sensor/pose_fixed_scale: true
position_pose_sensor/pose_fixed_p_ic: true
position_pose_sensor/pose_fixed_q_ic: true
position_pose_sensor/pose_fixed_p_wv: false
position_pose_sensor/pose_fixed_q_wv: false

position_pose_sensor/pose_noise_scale: 0.0

position_pose_sensor/pose_noise_p_wv: 0.001
position_pose_sensor/pose_noise_q_wv: 0.0025
position_pose_sensor/pose_noise_p_ic: 0.0
position_pose_sensor/pose_noise_q_ic: 0.0
position_pose_sensor/pose_delay: -0.1258408930047
position_pose_sensor/pose_noise_meas_p: """+str(pose_dev_p+BASE_POSE_DEV_P)+"""
position_pose_sensor/pose_noise_meas_q: """+str(pose_dev_q+BASE_POSE_DEV_Q)+"""

position_pose_sensor/position_noise_meas: """+str(position_dev)+"""

position_pose_sensor/position_delay: -0.01
position_pose_sensor/position_noise_p_ip: 0.00
position_pose_sensor/position_fixed_p_ip: true

position_pose_sensor/position_yaw_init: 0


#these params are read in pose_sensorhandler.hpp and therefore have prefix pose_sensor
#these params actually dont matter
pose_sensor/init/q_ic/x: 0.807264573826
pose_sensor/init/q_ic/y: -0.000354456866938
pose_sensor/init/q_ic/z: 0.590149775732
pose_sensor/init/q_ic/w: 0.00685743456288

pose_sensor/init/p_ic/x: 0.118867928682
pose_sensor/init/p_ic/y: -0.0292211469162
pose_sensor/init/p_ic/z: -0.062828441889
#until here

pose_sensor/pose_absolute_measurements: true
pose_sensor/pose_use_fixed_covariance: true
pose_sensor/pose_measurement_world_sensor: true  # selects if sensor measures its position w.r.t. world (true, e.g. Vicon) or the position of the world coordinate system w.r.t. the sensor (false, e.g. ethzasl_ptam

#these params are read in position_sensorhandler.hpp and therefore have prefix position_sensor
#########GPS PARAMETERS#######
position_sensor/init/p_ip/x: 0.06901
position_sensor/init/p_ip/y: -0.02781
position_sensor/init/p_ip/z: -0.12395

position_sensor/position_absolute_measurements: true
position_sensor/position_use_fixed_covariance: true
position_sensor/position_measurement_world_sensor: true 


#settings for outlierrejection
pose_sensor/enable_mah_outlier_rejection: true
pose_sensor/mah_threshold: 5.0

position_sensor/enable_mah_outlier_rejection: true
position_sensor/mah_threshold: 5.0

#settings for noise estimation
pose_sensor/enable_noise_estimation: true
#setting this closer to 1 will make it more longtime closer to 0 more shorttime
#should think about frequency when defining this
pose_sensor/noise_estimation_discount_factor: 0.99
pose_sensor/max_noise_threshold: 2

#settings for noise estimation
position_sensor/enable_noise_estimation: true
#setting this closer to 1 will make it more longtime closer to 0 more shorttime
#should think about frequency when defining this
position_sensor/noise_estimation_discount_factor: 0.999
position_sensor/max_noise_threshold: 5


#settings for divergence recovery

pose_sensor/enable_divergence_recovery: true
pose_sensor/divergence_rejection_limit: 20

pose_sensor/use_reset_to_pose: true

pose_sensor/use_transform_recovery: true
#choose same as initial noise estimate
pose_sensor/transform_recovery_noise_p: 0.002
pose_sensor/transform_recovery_noise_q: 0.025
pose_sensor/transform_anealing_steps: 200

#settings for divergence recovery
#this should be false unless you expect gps to recover to different frame or have poor initialization
position_sensor/enable_divergence_recovery: false
position_sensor/divergence_rejection_limit: 300


#settings for training set
position_sensor/create_training_set: true
position_sensor/path_to_training_set: """+str(POSITION_FEATURE_FILE)+"""

pose_sensor/create_training_set: true
pose_sensor/path_to_training_set: """+str(POSE_FEATURE_FILE))
	#for safety sleep a few seconds
	sleep(2)
	#call bash script to run model
	subprocess.check_call(["./collect_training.sh", str(launch_file), str(rosbag_file)])
	sleep(110)
	print("done with round")
	sleep(2)
	#wait long enough for run to finish (dont need this should be a blocking call)
	#sleep(120)
for i in range(N_RUNS):
	#select all possible parameters as randomly as possible
	noise_type = np.random.choice(NOISE_TYPE)
	#parameters for position
	#initial noise
	position_dev = np.random.uniform(P_MIN_NOISE, P_MAX_NOISE)
	#meta noise
	position_meta_noise_mean = np.random.uniform(-META_NOISE_MEAN_MAX, META_NOISE_MEAN_MAX)
	position_meta_noise_dev = np.random.uniform(0, META_NOISE_DEV_MAX)
	position_p_outlier = np.random.uniform(MIN_P_OUTLIER, MAX_P_OUTLIER)
	
	#parameters for pose
	#initial noise
	pose_dev_p = np.random.uniform(P_MIN_NOISE/5, P_MAX_NOISE/5)
	pose_dev_q = np.random.uniform(Q_MIN_NOISE, Q_MAX_NOISE)
	#pose_dev_q=0.0
	#meta noise
	pose_meta_noise_mean_p = np.random.uniform(-META_NOISE_MEAN_MAX, META_NOISE_MEAN_MAX)
	pose_meta_noise_dev_p = np.random.uniform(0, META_NOISE_DEV_MAX)
	pose_meta_noise_mean_q = np.random.uniform(-META_NOISE_MEAN_MAX/10, META_NOISE_MEAN_MAX/10)
	pose_meta_noise_dev_q = np.random.uniform(0, META_NOISE_DEV_MAX/10)
	#pose_meta_noise_mean_q = 0.0
	#pose_meta_noise_dev_q = 0.0
	pose_p_outlier = np.random.uniform(MIN_P_OUTLIER, MAX_P_OUTLIER)
	#run a single round
	run_single(noise_type, position_dev, position_meta_noise_mean, position_meta_noise_dev, position_p_outlier,
		pose_dev_p, pose_dev_q, pose_meta_noise_mean_p, pose_meta_noise_dev_p,
		pose_meta_noise_mean_q, pose_meta_noise_dev_q, pose_p_outlier, LAUNCH_FILE, ROSBAG_FILE)
