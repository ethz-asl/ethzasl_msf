import tensorflow as tf
import numpy as np
from data_loading import *

# To look at graph use:
# tensorboard --logdir ./Model/summ_BaselineModel/

# Choose a model, replace the model_name and the model_constructor accordingly
# to train and evaluate your own model.

###### simple LSTM model
model_name = "SimpleLSTM"
from lstm_simple import *
model_constructor = build_LSTM_simple_model


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
BATCHSIZE = 32
N_EPOCHS = 40
if DEBUG: 
	N_EPOCHS = 4
# Length of sequences used for training (i.e n measurements per line): need to create data reading
max_sequence_length = 10
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
print("\tMax. Input Question Length: " + str(max_sequence_length))
print("\tSave model parameters every: " + str(n_steps_per_save) + " steps.")
print("\tUsing saved model? " + str(use_stored_model))
print("")

########################################################################
# Setup Data
# Path into data folder
path_dataset = './Data'
	
# Choose Training and validation dataset
name_t = "trainval2014_train"
name_v = "trainval2014_val"


# Load all the needed data
[quest_vocab_dim, answer_vocab_size, 
					num_quests_train, quest_to_img_map_train, num_quests_val, 
					quest_to_img_map_val, train_quests, val_quests, answers_indxs_train,
					answers_indxs_val, all_answers, num_ans,
					n_img_features, data_train, data_val,img_dict
					] = load_all_the_data(path_dataset, path_images, max_sequence_length, name_t, name_v, name_train, name_val, first)
########################################################################
# Build the model
[train_step, loss_to_minimize, accuracy, feats_inpt, labels, training] = model_constructor(n_img_features, quest_vocab_dim, answer_vocab_size, max_sequence_length, batchsize = 32, seed = 42)

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
	n_step_tot = num_quests_train / BATCHSIZE
	step = sample_indx / BATCHSIZE
	for k in range(N_EPOCHS):
		
		# Initialize quantities
		n_batches = 0
		train_loss_epoch = 0.0
		accuracy_epoch = 0.0
		print("Starting epoch " + str(k+1) + " of " + str(N_EPOCHS))
		
		# Iterate over all questions
		while True:
			# Count how many batches
			n_batches += 1
			
			# Prepare labels and input
			if verbose:
				print("Epoch: " + str(k + 1) + " of " + str(N_EPOCHS) + ", Step: " + str(step+1) + " of " + str(n_step_tot))
				print("Preparing input...")
			if sample_indx + BATCHSIZE < num_quests_train:
				[batch_labels, batch_img_feats, batch_quest_feats, _] = prepare_input(sample_indx, num_quests_train, 
								BATCHSIZE, answers_indxs_train, answer_vocab_size, n_img_features,
								max_sequence_length, quest_to_img_map_train, data_train, data_val, name_train, name_val,
								train_quests, img_dict, all_answers, quest_vocab_dim)
			else:
				break
			
			# Do one train step
			if verbose:
				print("Doing a training step...")
			# Do a training step and return the loss
			_, train_loss, summary, accuracy_curr = sess.run([train_step, loss_to_minimize, merged_summary_op, accuracy], 
							feed_dict={img_feats_inpt: batch_img_feats, 
									  quest_feats_inpt: batch_quest_feats, 
									  labels: batch_labels,
									  training: True,
									  })
			if verbose:  
				print("Loss: " + str(train_loss))
			train_loss_epoch += train_loss
			accuracy_epoch += accuracy_curr
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
			f.write(str(train_loss_epoch) + "," + str(accuracy_epoch / n_batches) + "\n")
		# Write accuracy to summary
		summary_writer.add_summary(summary, k)
		print("Training error of this epoch: " + str(train_loss_epoch))
		# Reset indices
		sample_indx = 0
		step = 0
		
	########################################################################
	# Evaluate the model on the validation data
	print("\nEvaluating model..")
	sample_indx = 0
	# Iterate over all questions
	n_batches = 0
	n_bat_tot = num_quests_val / BATCHSIZE
	accur = 0.0
	accur_human = 0.0
	while True:
		# Prepare labels and input
		if verbose:
			print("Step: " + str(n_batches) + " of " + str(n_bat_tot))
			print("Preparing input...")
		if sample_indx + BATCHSIZE < num_quests_val:
			[batch_labels, batch_img_feats, batch_quest_feats, batch_all_ans] = prepare_input(sample_indx, num_quests_val, 
							BATCHSIZE, answers_indxs_val, answer_vocab_size, n_img_features,
							max_sequence_length, quest_to_img_map_val, data_train, data_val, name_train, name_val,
							val_quests, img_dict, all_answers, quest_vocab_dim, training = False)
		else:
			break
		
		# Do one evaluation step
		if verbose:
			print("Doing an evaluation step...")
		accur_curr, modif_accur_curr = sess.run([accuracy, modif_accur],
							feed_dict={img_feats_inpt: batch_img_feats, 
									  quest_feats_inpt: batch_quest_feats, 
									  labels: batch_labels,
									  all_answers_input: batch_all_ans,
									  training: False,
									  })
		accur += accur_curr
		accur_human += modif_accur_curr

		if verbose:
			print("The accuracy of this batch is: " + str(accur_curr))
			print("The evaluation accuracy of this batch is: " + str(modif_accur_curr))
		
		n_batches += 1
		sample_indx += BATCHSIZE
		
		if DEBUG:
			# Only evaluate on first three batches
			if n_batches == 3:
				break
	
	print("The accuracy on the evaluation set is: " + str(accur / n_batches))
	print("The evaluation accuracy is: " + str(accur_human / n_batches))







