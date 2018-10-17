import keras
from keras.layers import LSTM, RNN, TimeDistributed, Dense
from keras.models import Sequential
import numpy as np


def build_keras_LSTM_model(n_features, num_noise_params, max_sequence_length, initial_state=0, seed = 42, train = True):
	model = Sequential()
	model.add(LSTM(16, return_sequences=True, input_shape=(None,n_features)))
	model.add(LSTM(8, return_sequences=False))
	model.add(Dense(num_noise_params))
	return model

