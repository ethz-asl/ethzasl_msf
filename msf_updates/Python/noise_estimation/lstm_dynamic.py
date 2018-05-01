import tensorflow as tf
import tensorflow.contrib.layers as layers
import numpy as np

map_fn = tf.map_fn

def build_LSTM_dynamic_model(n_features, num_noise_params, max_sequence_length, initial_state=0, seed = 42, train = True):
	
	TINY = 1e-6 
	
	# Model architecture params
	LEARNING_RATE = 0.01
	dropout_rate_input = 0.2
	dropout_rate_hidden = 0.5
	INPUT_SIZE = n_features
	n_hidden_RNN = 81
	OUTPUT_SIZE = num_noise_params
	#hidden_fcl_sizes = [512, 512]
	activ = tf.nn.sigmoid
	regularization_param = 1.0
	
	print("Network Parameters:")
	print("\tLearning Rate: " + str(LEARNING_RATE))
	#print("\tHidden layer sizes: " + str(hidden_fcl_sizes))
	print("\tDropout probability for input: " + str(dropout_rate_input))
	print("\tDropout probability for hidden layers: " + str(dropout_rate_hidden))
	print("")
	
	########################################################################
	# Define the Model

	# Inputs Paceholders
	feats_inpt = tf.placeholder(tf.float32, shape = [None, None, INPUT_SIZE], name = "input_features")
	
	labels = tf.placeholder(tf.float32, shape = [None, None, OUTPUT_SIZE], name = "input_labels")
	
	drop_prob_input = dropout_rate_input
	drop_prob_hidden = dropout_rate_hidden

	# Refularization and Initialization
	tf.set_random_seed(seed)
	regularizer = tf.contrib.layers.l2_regularizer(scale = regularization_param)
	norm_initializer = tf.contrib.layers.xavier_initializer(uniform=False, dtype=tf.float32)


	# The LSTM with additional dropout
	rnn_cell = tf.contrib.rnn.BasicLSTMCell(n_hidden_RNN, state_is_tuple=True)


	initial_state = rnn_cell.zero_state(tf.shape(feats_inpt)[1], tf.float32)
	
	rnn_outputs, state = tf.nn.dynamic_rnn(rnn_cell, feats_inpt, initial_state=initial_state, dtype=tf.float32, time_major=True) #output is rnn_outputs, rnn_states

	
	final_projection = lambda x: tf.contrib.layers.linear(x, num_outputs=num_noise_params, activation_fn=activ)
	
	# apply projection to every timestep.
	predicted_outputs = map_fn(final_projection, rnn_outputs, name="predictions")
	
	prediction = tf.identity(predicted_outputs, name="prediction")
	#tf.summary.tensor_summary(name="predictions", tensor=predicted_outputs)

	#with tf.name_scope("cross_entropy_ns"):
	# compute elementwise cross entropy.
	#cross_entropy = -(labels * tf.log(predicted_outputs + TINY) + (1.0 - labels) * tf.log(1.0 - predicted_outputs + TINY))
	#error = tf.reduce_mean(cross_entropy, name="error")
	error = tf.losses.mean_squared_error(labels, predicted_outputs)
	tf.summary.scalar("error", error)

	final_state=state
	
	# Define loss and optimizer
	train_step = tf.train.RMSPropOptimizer(LEARNING_RATE).minimize(error)

	# Return training step, loss, predictions, accuracy and input placeholders
	return [train_step, error, feats_inpt, labels, predicted_outputs]












