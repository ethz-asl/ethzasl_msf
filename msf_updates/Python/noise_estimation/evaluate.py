import tensorflow as tf
import numpy as np
from data_processing import *
import sys
from math import floor

# To look at graph use:
# tensorboard --logdir ./Model/summ_BaselineModel/

# Choose a model, replace the model_name and the model_constructor accordingly
# to train and evaluate your own model.

###### simple LSTM model
#model_name = "SimpleLSTM"
#from lstm_simple import *
#model_constructor = build_LSTM_simple_model

model_name = "DynamicLSTM"
from lstm_dynamic import *
model_constructor = build_LSTM_dynamic_model

if len(sys.argv)!=3:
	print("usage python evaluate.py features_file label_file")
	sys.exit()
featfile=sys.argv[1]
labelfile=sys.argv[2]



########################################################################
# Debugging? (Use True on local machine.)
DEBUG = True
# Where to store the model 
model_path = "./Model/"
# Whether to use the stored model
# Throws an error if required model does not exist
# CAUTION: you may overwrite parameters of another model.
use_stored_model = True
# Hyperparameters (global constants)
BATCHSIZE = 1
N_EPOCHS = 40
if DEBUG: 
	N_EPOCHS = 4
# Length of sequences used for training (i.e n measurements per line): need to create data reading
#this is for GPS
max_sequence_length = 100
#this is for Pose
#max_sequence_length = 200

# Save options
n_steps_per_save = 500
if DEBUG: 
	n_steps_per_save = 3
# Generate output in DEBUG mode
verbose = DEBUG
SEED = 42
np.random.seed(SEED)

print("General parameters:")
print("\tNumber of epochs: " + str(N_EPOCHS))
print("\tBatch Size: " + str(BATCHSIZE))
print("\tMax. Sequence Length: " + str(max_sequence_length))
print("\tSave model parameters every: " + str(n_steps_per_save) + " steps.")
print("\tUsing saved model? " + str(use_stored_model))
print("")

########################################################################
# Setup Data
#print(featfile)
[features_train, labels_train, features_val, labels_val] = load_all_data(featfile, labelfile, max_sequence_length)
num_train = features_train.shape[1]
num_val = features_val.shape[1]
n_sequence_per_run = features_train.shape[1]

########################################################################
# Build the model
[train_step, loss_to_minimize, feats_inpt, labels, predictions] = model_constructor(features_train.shape[2], labels_train.shape[2], max_sequence_length, batchsize = BATCHSIZE, seed = 42, train=False)

# Count the number of trainable parameters
count_pars()

# Set verbosity
tf.logging.set_verbosity(tf.logging.INFO)

# Do summary stuff
merged_summary_op = tf.summary.merge_all()

########################################################################
# Train the model
print("Training the model: " + model_name + "...")

# Define a saver
saver = tf.train.Saver()

with tf.Session() as sess:
	
	# Prepare summary writing
	summ_filename = model_path + "summ_" + model_name
	summary_writer = tf.summary.FileWriter(summ_filename, graph=tf.get_default_graph())
	
	########################################################################
	# Evaluate the model on the validation data
	print("\nEvaluating model..")
	sample_indx = 0
	# Iterate over all questions
	n_batches = 0
	n_bat_tot = num_val / BATCHSIZE
	error = 0.0
	while True:
		# Prepare labels and input
		if verbose:
			print("Step: " + str(n_batches) + " of " + str(n_bat_tot))
			print("Preparing input...")
		
		#create batch
		if sample_indx + BATCHSIZE <= num_val:
			[features_val_batch, labels_val_batch] = prepare_input(sample_indx, BATCHSIZE, features_val, labels_val)
		else:
			break
			
		# Do one evaluation step
		if verbose:
			print("Doing an evaluation step...")
		[error_curr, prediction_curr] = sess.run([loss_to_minimize, predictions],
							feed_dict={feats_inpt: features_val_batch, 
									  labels: labels_val_batch,
									  #training: False,
									  })
		error += error_curr

		if verbose:
			print("predicting: "+str(prediction_curr))
			print("The MSE on this batch is: " + str(error_curr))
		
		n_batches += 1
		sample_indx += BATCHSIZE
		
		if DEBUG:
			# Only evaluate on first three batches
			if n_batches == 3:
				break
	
	print("The error on the evaluation set is: " + str(error / n_batches))








