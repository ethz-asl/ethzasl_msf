import keras
from keras.layers import LSTM, RNN
from keras.models import Sequential
from keras import optimizers
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

"""
model_name = "KerasLSTM"
from keras_lstm import *
model_constructor = build_keras_LSTM_model
"""
model_name = "KerasLSTM2"
from keras_lstm2 import *
model_constructor = build_keras_LSTM_model

if len(sys.argv)!=3:
	print("usage python train_and_evaluate features_file label_file")
	sys.exit()
featfile=sys.argv[1]
labelfile=sys.argv[2]



########################################################################
# Debugging? (Use True on local machine.)
DEBUG = False
# Where to store the model 
model_path = "./Model/"
# Whether to use the stored model
# Throws an error if required model does not exist
# CAUTION: you may overwrite parameters of another model.
use_stored_model = False
# Hyperparameters (global constants)
BATCHSIZE = 128
N_EPOCHS = 100
if DEBUG: 
	N_EPOCHS = 4
# Length of sequences used for training (i.e n measurements per line): need to create data reading
#this is for GPS
max_sequence_length = 100
#this is for Pose
#max_sequence_length = 20

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
#here data is sequencelength, ntraining, ndim
[features_train, labels_train, features_val, labels_val] = load_all_data(featfile, labelfile, max_sequence_length)
features_train = np.einsum('ijk->jik', features_train)
labels_train = np.einsum('ijk->jik', labels_train)[:,-1,:]
features_val = np.einsum('ijk->jik', features_val)
labels_val = np.einsum('ijk->jik', labels_val)[:,-1,:]

kerasmodel = model_constructor(features_train.shape[2], labels_train.shape[1], max_sequence_length, seed = 42)

########################################################################
# Train the model
print("Training the model: " + model_name + "...")

kerasopt=optimizers.Adam(lr=0.001)
kerasmodel.compile(optimizer=kerasopt,loss='mse')
for i in range(20):
	kerasmodel.fit(features_train, labels_train, epochs=10, verbose=DEBUG, batch_size=BATCHSIZE, validation_data=(features_val, labels_val))
	score=kerasmodel.evaluate(features_val,labels_val)
	print(score)

kerasmodel.save(model_path+model_name+'.h5')
		
########################################################################
# Evaluate the model on the validation data
score = kerasmodel.evaluate(features_val, labels_val)
print(score)





