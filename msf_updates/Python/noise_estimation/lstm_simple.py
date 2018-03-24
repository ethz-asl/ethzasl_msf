import tensorflow as tf

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
	print("\tHidden layer sizes: " + str(hidden_fcl_sizes))
	print("\tDropout probability for input: " + str(dropout_rate_input))
	print("\tDropout probability for hidden layers: " + str(dropout_rate_hidden))
	print("")
	
	########################################################################
	# Define the Model

	# Inputs Paceholders
	feats_inpt = tf.placeholder(tf.float32, shape = [max_sequence_length, batchsize, n_features], name = "input_features")

	labels = tf.placeholder(tf.int64, shape = [batchsize], name = "input_labels_integer_encoded")
	all_answers_input = tf.placeholder(tf.int64, shape = [batchsize, 10], name = "input_all_answers")
	drop_prob_input = dropout_rate_input
	drop_prob_hidden = dropout_rate_hidden

	# Refularization and Initialization
	tf.set_random_seed(seed)
	regularizer = tf.contrib.layers.l2_regularizer(scale = regularization_param)
	norm_initializer = tf.contrib.layers.xavier_initializer(uniform=False, dtype=tf.float32)


	# The LSTM with additional dropout
	rnn_cell = tf.contrib.rnn.BasicLSTMCell(n_hidden_RNN)
	rnn_outputs, _ = tf.contrib.rnn.static_rnn(rnn_cell, feats_inpt, dtype=tf.float32) #output is rnn_outputs, rnn_states
	#rnn_outputs_drop = tf.nn.dropout(rnn_outputs[-1], drop_prob_input, name = "rnn_output_dropout")

	# The Fully connected hidden layers
	#n_hidd_l = len(hidden_fcl_sizes)

	# Concatenate the features


	# Dense hidden layers
	"""for k in range(n_hidd_l):
		# Use dense layers
		layer_name = "dense_hidden_layer_" + str(k+1)
		dropout_name = "dense_hidden_layer_dropout" + str(k+1)
		with tf.name_scope(layer_name):
			fcl = tf.layers.dense(fcl_drop_nonl, hidden_fcl_sizes[k],
							name = layer_name,
							activation=activ,
							kernel_initializer = norm_initializer,
							kernel_regularizer = regularizer)
			fcl_drop_nonl = tf.nn.dropout(fcl, drop_prob_hidden, name = dropout_name)
	"""
	
	# The output, a dense layer without activation and a skip conection not using a bias for the second layer because of redundancy
	out_acts = tf.layers.dense(rnn_outputs, num_noise_params, name = "output_activations_part")
	predictions = out_acts
	#predictions = tf.nn.softmax(out_acts, name = "output_softmax")
	# Use a namescope here to make the graph much much more nice to look at
	with tf.name_scope("cross_entropy_ns"):
		cross_entropy = tf.reduce_mean(-(label * tf.log(predictions + TINY) + (1.0 - label) * tf.log(1.0 - predictions + TINY)))
	accuracy = tf.reduce_mean(tf.abs(label - predictions), tf.float32)
	
	
	tf.summary.scalar("accuracy", accuracy)
	
	# Define loss and optimizer
	train_step = tf.train.RMSPropOptimizer(LEARNING_RATE).minimize(cross_entropy)

	# Return training step, loss, predictions, accuracy and input placeholders
	return [train_step, cross_entropy, accuracy, feats_inpt, labels]












