import tensorflow as tf

map_fn = tf.map_fn

def build_LSTM_simple_model(n_features, num_noise_params, max_sequence_length, batchsize = 32, seed = 42):
	
	TINY = 1e-6 
	
	# Model architecture params
	LEARNING_RATE = 0.01
	dropout_rate_input = 0.2
	dropout_rate_hidden = 0.5
	n_hidden_RNN = 81
	n_out_RNN = n_features
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
	feats_inpt = tf.placeholder(tf.float32, shape = [max_sequence_length, batchsize, n_features], name = "input_features")
	#feats_inpt_list = tf.split(feats_inpt, num_or_size_splits=max_sequence_length, axis=0)
	#for idx, fi in enumerate(feats_inpt_list):
	#	feats_inpt_list[idx]=fi[0,:,:]
	#print(feats_inpt_list[0].shape)
	labels = tf.placeholder(tf.float32, shape = [max_sequence_length, batchsize, num_noise_params], name = "input_labels")
	
	drop_prob_input = dropout_rate_input
	drop_prob_hidden = dropout_rate_hidden

	# Refularization and Initialization
	tf.set_random_seed(seed)
	regularizer = tf.contrib.layers.l2_regularizer(scale = regularization_param)
	norm_initializer = tf.contrib.layers.xavier_initializer(uniform=False, dtype=tf.float32)


	# The LSTM with additional dropout
	rnn_cell = tf.contrib.rnn.BasicLSTMCell(n_hidden_RNN)
	rnn_outputs, _ = tf.nn.dynamic_rnn(rnn_cell, feats_inpt, dtype=tf.float32, time_major=True) #output is rnn_outputs, rnn_states

	
	final_projection = lambda x: tf.contrib.layers.linear(x, num_outputs=num_noise_params, activation_fn=activ)
	
	# apply projection to every timestep.
	predicted_outputs = map_fn(final_projection, rnn_outputs)


	#with tf.name_scope("cross_entropy_ns"):
	# compute elementwise cross entropy.
	cross_entropy = -(labels * tf.log(predicted_outputs + TINY) + (1.0 - labels) * tf.log(1.0 - predicted_outputs + TINY))
	error = tf.reduce_mean(cross_entropy, name="error")
	tf.summary.scalar("error", error)

	
	# Define loss and optimizer
	train_step = tf.train.RMSPropOptimizer(LEARNING_RATE).minimize(error)

	# Return training step, loss, predictions, accuracy and input placeholders
	return [train_step, cross_entropy, error, feats_inpt, labels]












