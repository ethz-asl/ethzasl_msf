import numpy as np
import tensorflow as tf
import os
from random import shuffle
"""
#input: path to file containing features and labels. these files are supposed to have same #lines
#to assure run postprocess_ts.py
#sequence length is the sequence length of the training data
def load_all_data(feature_file, label_file, sequence_length):
	NVAL=8 #number of RUNS that are selected for evaluation
	#read files do preprocessing: split file sequences into actual sequence length
	featf = open(feature_file, "r")
	labelf = open(label_file, "r")
	sequence_list_f = []
	sequence_list_l = []
	list_of_is=[]
	featlines = featf.readlines()
	labellines = labelf.readlines()
	#0 is separator
	idx = 1
	nruns = 0
	while idx<len(featlines):#this is over runs
		sequence_list_f.append([])
		sequence_list_l.append([])
		nsequence = 0
		newrun = False
		while idx<len(featlines): #this is over sequences: a run is split into several sequences that are fed in order to the lstm
			sequence_list_f[nruns].append([])
			sequence_list_l[nruns].append([])
			i = 0
			skip = 0
			while idx+i<len(featlines) and i<sequence_length: #this is iteration over one sequence (one split of a run)
				if featlines[idx+i]=="starting new training sequence\n":
					skip = 1
					#remove incomplete list parts
					sequence_list_f[nruns].pop()
					sequence_list_l[nruns].pop()
					nsequence-=1
					newrun = True
					break
				templist_f=featlines[idx+i].rstrip('\n').split(',')
				templist_l=labellines[idx+i].rstrip('\n').split(',')
				sequence_list_f[nruns][nsequence].append([])
				for fnum in templist_f:
					sequence_list_f[nruns][nsequence][i].append(float(fnum))
				sequence_list_l[nruns][nsequence].append([])
				for lnum in templist_l:
					sequence_list_l[nruns][nsequence][i].append(float(lnum))
				i+=1
			if skip==0:
				list_of_is.append(i)
			nsequence +=1
			idx+=i+skip
			if newrun:
				#in case there are empty runs for some reason
				if nsequence==0:
					sequence_list_f.pop()
					sequence_list_l.pop()
					nruns-=1
				nruns+=1
				break
	#remove last element of both lists since it will be incomplete again
	sequence_list_f[-1].pop()
	sequence_list_l[-1].pop()
	#combined = list(zip(sequence_list_f, sequence_list_l))
	#shuffle(combined)


	#dimension of features/labels are: n_runs, n_sequences_per_run, sequence_length, n_features
	#we probably have non matching dimensions....how to fix this....
	#dimension missmatch can happen in number of sequences per run since they might slightly vary
	#reduce to smallest
	min_n_seq=min(len(sequence_list_f[i]) for i in range(len(sequence_list_f)))
	for i in range(len(sequence_list_f)):
		for j in range(len(sequence_list_f[i])-min_n_seq):
			sequence_list_f[i].pop()
			sequence_list_l[i].pop()
	features=np.asarray(sequence_list_f, dtype=np.float32)
	labels=np.asarray(sequence_list_l, dtype=np.float32)
	print(features.shape)
	#need to split into train and val
	return [features[NVAL:, :, :, :], labels[NVAL:,:, :, :], features[:NVAL, :, :, :], labels[:NVAL,:, :, :]]
"""
#input: path to file containing features and labels. these files are supposed to have same #lines
#to assure run postprocess_ts.py
#sequence length is the sequence length of the training data
def load_all_data(feature_file, label_file, sequence_length):
	NVAL=24
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
	while idx<len(featlines) and idx<len(labellines):
		sequence_list_f.append([])
		sequence_list_l.append([])
		i = 0
		skip = 0
		while idx+i<len(featlines) and idx+i<len(labellines) and i<sequence_length:
			if featlines[idx+i]=="starting new training sequence\n" or labellines[idx+i]=="starting new training sequence\n":
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
	
	sequence_list_f[:], sequence_list_l[:] = zip(*combined)
	#sequence_list_f[:], sequence_list_l[:] = zip(*combined)
	#print(sequence_list_f)

	features=np.einsum('ijk->jik', np.asarray(sequence_list_f, dtype=np.float32))
	labels=np.einsum('ijk->jik', np.asarray(sequence_list_l, dtype=np.float32))
	#need to split into train and val
	return [features[:, NVAL:, :], labels[:,NVAL:,:], features[:, :NVAL, :], labels[:,:NVAL,:]]


"""
# Here the input and the labels of the model is prepared. 
# We take a batch for all data (i.e features and labels)
#for features and for labels it is axis=1
#input is a 4d tensor with shape nruns, n_sequences_per_run, sequence_length, features
#output is a 4d tensor with shape batchsize, n_sequences_per_run, sequence_length, features
def prepare_input(sample_indx, batchsize, feature_inpt, label_inpt, training = True):
	labels_batch = np.zeros((batchsize, label_inpt.shape[1], label_inpt.shape[2],label_inpt.shape[3]), dtype = np.float32)
	features_batch = np.zeros((batchsize, feature_inpt.shape[1], feature_inpt.shape[2], feature_inpt.shape[3]), dtype = np.float32)
	bat_count = 0
	while bat_count < batchsize:
		labels_batch[bat_count, :,:,:]=label_inpt[sample_indx,:,:,:]
		features_batch[bat_count,:,:,:]=feature_inpt[sample_indx,:,:,:]
			
		bat_count += 1
		sample_indx += 1
	features_batch=np.einsum('ijkl->kjil', features_batch)
	labels_batch=np.einsum('ijkl->kjil', labels_batch)
	return [features_batch, labels_batch]
"""
	
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
	
	
