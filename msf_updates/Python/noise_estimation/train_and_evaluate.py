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
	print("usage python train_and_evaluate features_file label_file")
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
use_stored_model = False
# Hyperparameters (global constants)
BATCHSIZE = 8
N_EPOCHS = 40
if DEBUG: 
	N_EPOCHS = 4
# Length of sequences used for training (i.e n measurements per line): need to create data reading
#this is for GPS
max_sequence_length = 100
#this is for Pose
#max_sequence_length = 20
# If question is longer use first part (True) or last part (False)
first = True
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
[train_step, loss_to_minimize, accuracy, feats_inpt, labels] = model_constructor(features_train.shape[2], labels_train.shape[2], max_sequence_length, batchsize = BATCHSIZE, seed = 42)

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

"""
with tf.Session() as sess:
	
	# Prepare summary writing
	summ_filename = model_path + "summ_" + model_name
	summary_writer = tf.summary.FileWriter(summ_filename, graph=tf.get_default_graph())
	
	# Clear training error file
	with open(model_path + model_name + "_curr_loss.txt", "w") as f:
		pass
	
	sample_indx = 0
	if use_stored_model:
		saver.restore(sess, model_path + model_name + ".ckpt")
		with open(model_path + model_name + "_info.txt", "r") as f:
			sample_indx = int(f.read())
	else:
		sess.run(tf.global_variables_initializer())
	
	
	# Iterate over all epochs
	print("\nStarting training...")
	n_runs_batch = int(floor(num_train_runs/BATCHSIZE))
	step = sample_indx / BATCHSIZE
	for k in range(N_EPOCHS):
		# Initialize quantities
		n_batches = 0
		train_loss_epoch = 0.0
		error_epoch = 0.0
		#iterate over batches of sequences
	
		#reinserted
		print("Starting epoch " + str(k+1) + " of " + str(N_EPOCHS))
		for i in range(n_runs_batch):
			# Iterate over whole sequence length (features)
			#create batch
			if sample_indx + BATCHSIZE <= num_train_runs:
				[features_train_batch, labels_train_batch] = prepare_input(sample_indx, BATCHSIZE, features_train, labels_train)
			else:
				break
			num_sequence_iterations = features_train_batch.shape[1]
			# Count how many batches
			n_batches += 1
			for j in range(num_sequence_iterations): #this iterates over the same batch of sequences but goes through them
				
				
				# Prepare labels and input
				if verbose:
					print("Epoch: " + str(k + 1) + " of " + str(N_EPOCHS) + ", Step: " + str(step+1) + " of " + str(n_runs_batch))
					print("Preparing input...")
				
				# Do one train step
				if verbose:
					print("Doing a training step...")
				
				
				# Do a training step and return the loss
				_, train_loss, error_curr, summary, final_state = sess.run([train_step, loss_to_minimize, accuracy, merged_summary_op, final_state], 
								feed_dict={feats_inpt: features_train_batch[:,j, :,:], 
										  labels: labels_train_batch[:,j,:,:],
										  #training: True,
										  })
				if verbose:  
					print("Loss: " + str(error_curr))
					#print("train loss: " + str(train_loss))
				train_loss_epoch += train_loss
				error_epoch += error_curr
			step += 1
			sample_indx += BATCHSIZE
			
			# Save model
			if not (step % n_steps_per_save):
				if verbose:
					print("Saving parameters...")
				with open(model_path + model_name + "_info.txt", "w") as f:
					f.write(str(sample_indx))
				save_path = saver.save(sess, model_path + model_name + ".ckpt")
				
			if DEBUG:
				# Only do 5 steps:
				if step == 5:
					break
		# Write loss and accuracy to file to observe training progress
		with open(model_path + model_name + "_curr_loss.txt", "a") as f:
			f.write(str(train_loss_epoch) + "," + str(error_epoch / n_batches) + "\n")
		# Write accuracy to summary
		summary_writer.add_summary(summary, k)
		#print("Training error of this epoch: " + str(train_loss_epoch))
		# Reset indices
		sample_indx = 0
		step = 0
		
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
			print("Step: " + str(n_batches) + " of " + str(n_runs_batch))
			print("Preparing input...")
		
		#create batch
		if sample_indx + BATCHSIZE <= num_val:
			[features_val_batch, labels_val_batch] = prepare_input(sample_indx, BATCHSIZE, features_val, labels_val)
		else:
			break
			
		# Do one evaluation step
		if verbose:
			print("Doing an evaluation step...")
		[error_curr] = sess.run([accuracy],
							feed_dict={feats_inpt: features_val_batch, 
									  labels: labels_val_batch,
									  #training: False,
									  })
		error += error_curr

		if verbose:
			print("The accuracy of this batch is: " + str(error_curr))
		
		n_batches += 1
		sample_indx += BATCHSIZE
		
		if DEBUG:
			# Only evaluate on first three batches
			if n_batches == 3:
				break
	
	print("The accuracy on the evaluation set is: " + str(error / n_batches))
"""

with tf.Session() as sess:
	
	# Prepare summary writing
	summ_filename = model_path + "summ_" + model_name
	summary_writer = tf.summary.FileWriter(summ_filename, graph=tf.get_default_graph())
	
	# Clear training error file
	with open(model_path + model_name + "_curr_loss.txt", "w") as f:
		pass
	
	sample_indx = 0
	if use_stored_model:
		saver.restore(sess, model_path + model_name + ".ckpt")
		with open(model_path + model_name + "_info.txt", "r") as f:
			sample_indx = int(f.read())
	else:
		sess.run(tf.global_variables_initializer())
	
	# Iterate over all epochs
	print("\nStarting training...")
	n_step_tot = num_train / BATCHSIZE
	step = sample_indx / BATCHSIZE
	for k in range(N_EPOCHS):
		
		# Initialize quantities
		n_batches = 0
		train_loss_epoch = 0.0
		error_epoch = 0.0
		print("Starting epoch " + str(k+1) + " of " + str(N_EPOCHS))
		
		# Iterate over all sequences (features)
		while True:
			# Count how many batches
			n_batches += 1
			
			# Prepare labels and input
			if verbose:
				print("Epoch: " + str(k + 1) + " of " + str(N_EPOCHS) + ", Step: " + str(step+1) + " of " + str(n_step_tot))
				print("Preparing input...")
			
			# Do one train step
			if verbose:
				print("Doing a training step...")
			
			#create batch
			if sample_indx + BATCHSIZE <= num_train:
				[features_train_batch, labels_train_batch] = prepare_input(sample_indx, BATCHSIZE, features_train, labels_train)
			else:
				break
			# Do a training step and return the loss
			_, train_loss, error_curr, summary = sess.run([train_step, loss_to_minimize, accuracy, merged_summary_op], 
							feed_dict={feats_inpt: features_train_batch, 
									  labels: labels_train_batch,
									  #training: True,
									  })
			if verbose:  
				print("Loss: " + str(error_curr))
				#print("train loss: " + str(train_loss))
			train_loss_epoch += train_loss
			error_epoch += error_curr
			step += 1
			sample_indx += BATCHSIZE
			
			# Save model
			if not (step % n_steps_per_save):
				if verbose:
					print("Saving parameters...")
				with open(model_path + model_name + "_info.txt", "w") as f:
					f.write(str(sample_indx))
				save_path = saver.save(sess, model_path + model_name + ".ckpt")
				
			if DEBUG:
				# Only do 5 steps:
				if step == 5:
					break
		# Write loss and accuracy to file to observe training progress
		with open(model_path + model_name + "_curr_loss.txt", "a") as f:
			f.write(str(train_loss_epoch) + "," + str(error_epoch / n_batches) + "\n")
		# Write accuracy to summary
		summary_writer.add_summary(summary, k)
		#print("Training error of this epoch: " + str(train_loss_epoch))
		# Reset indices
		sample_indx = 0
		step = 0
		
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
		[error_curr] = sess.run([accuracy],
							feed_dict={feats_inpt: features_val_batch, 
									  labels: labels_val_batch,
									  #training: False,
									  })
		error += error_curr

		if verbose:
			print("The accuracy of this batch is: " + str(error_curr))
		
		n_batches += 1
		sample_indx += BATCHSIZE
		
		if DEBUG:
			# Only evaluate on first three batches
			if n_batches == 3:
				break
	
	print("The accuracy on the evaluation set is: " + str(error / n_batches))








