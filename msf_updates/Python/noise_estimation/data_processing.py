import numpy as np
import tensorflow as tf
import os
from random import shuffle

#input: path to file containing features and labels. these files are supposed to have same #lines
#to assure run postprocess_ts.py
#sequence length is the sequence length of the training data
def load_all_data(feature_file, label_file, sequence_length):
	NVAL=8
	#read files do preprocessing: split file sequences into actual sequence length
	featf = open(feature_file, "r")
	labelf = open(label_file, "r")
	sequence_list_f = []
	sequence_list_l = []
	featlines = featf.readlines()
	labellines = labelf.readlines()
	#0 is separator
	idx = 1
	nsequence = 0
	while idx<len(featlines):
		sequence_list_f.append([])
		sequence_list_l.append([])
		i = 0
		skip = 0
		while idx+i<len(featlines) and i<sequence_length:
			if featlines[idx+i]=="starting new training sequence\n":
				skip = 1
				#remove incomplete list parts
				sequence_list_f.pop()
				sequence_list_l.pop()
				nsequence-=1
				break
			templist_f=featlines[idx+i].rstrip('\n').split(',')
			templist_l=labellines[idx+i].rstrip('\n').split(',')
			sequence_list_f[nsequence].append([])
			for fnum in templist_f:
				sequence_list_f[nsequence][i].append(float(fnum))
			sequence_list_l[nsequence].append([])
			for lnum in templist_l:
				sequence_list_l[nsequence][i].append(float(lnum))
			i+=1
		nsequence +=1
		idx+=i+skip
	#remove last element of both lists since it will be incomplete again
	sequence_list_f.pop()
	sequence_list_l.pop()
	combined = list(zip(sequence_list_f, sequence_list_l))
	shuffle(combined)

	#sequence_list_f[:], sequence_list_l[:] = zip(*combined)
	#print(sequence_list_f)

	features=np.einsum('ijk->jik', np.asarray(sequence_list_f, dtype=np.float32))
	labels=np.einsum('ijk->jik', np.asarray(sequence_list_l, dtype=np.float32))
	#need to split into train and val
	return [features[:, NVAL:, :], labels[:,NVAL:,:], features[:, :NVAL, :], labels[:,:NVAL,:]]


# Here the input and the labels of the model is prepared. 
# We take a batch for all data (i.e features and labels)
#for features and for labels it is axis=1
def prepare_input(sample_indx, batchsize, feature_inpt, label_inpt, training = True):
	labels_batch = np.zeros((label_inpt.shape[0], batchsize,label_inpt.shape[2]), dtype = np.float32)
	features_batch = np.zeros((feature_inpt.shape[0], batchsize, feature_inpt.shape[2]), dtype = np.float32)
	bat_count = 0
	while bat_count < batchsize:
		labels_batch[:,bat_count,:]=label_inpt[:,sample_indx,:]
		features_batch[:,bat_count,:]=feature_inpt[:,sample_indx,:]
			
		bat_count += 1
		sample_indx += 1
	
	return [features_batch, labels_batch]
	

# Count the trainable parameters of the tensorflow model
def count_pars():
	# Count the number of trainable parameters
	total_parameters = 0
	for variable in tf.trainable_variables():
		# shape is an array of tf.Dimension
		shape = variable.get_shape()
		variable_parameters = 1
		for dim in shape:
			variable_parameters *= dim.value
		total_parameters += variable_parameters
		
	print("Built the network architecture with a total of " + str(total_parameters) + " parameters.\n")	
	
	
