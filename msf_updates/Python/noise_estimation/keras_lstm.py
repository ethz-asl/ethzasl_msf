import keras
from keras.layers import LSTM, RNN
from keras.models import Sequential
import numpy as np


def build_keras_LSTM_model(n_features, num_noise_params, max_sequence_length, initial_state=0, seed = 42, train = True):
	model = Sequential()
	model.add(LSTM(num_noise_params, return_sequences=True, input_shape=(100,3)))
	return model

