#!/bin/env python3

import os.path

import pred_data

import pickle

from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers.embeddings import Embedding
from keras.preprocessing import sequence

from keras.losses import mean_squared_error

import numpy as np

import matplotlib.pyplot as plt
# fix random seed for reproducibility
#  np.random.seed(7)

batch_size = 20
f_delay = 6
saved_batch_size = None

filename = "train_data.pickle"
if os.path.exists(filename):
    (saved_batch_size, train_x, train_y) = pickle.load(open(filename, "rb"))

if saved_batch_size != batch_size:
    print("Didn't find cached copy of training data, generating")
    (train_x, train_y) = pred_data.get_data(frame_delay=f_delay, window_size=batch_size)
    pickle.dump((batch_size, train_x, train_y), open(filename, "wb"))
else:
    print("Found cached copy with batch size = {}".format(saved_batch_size))

train_x = np.dstack(train_x)
train_x = np.swapaxes(train_x, 1, 2)
train_x = np.swapaxes(train_x, 0, 1)

train_y = np.matrix(train_y)

model = Sequential()

model.add(LSTM(50, return_sequences=True,
               input_shape=(batch_size, 7)))  # returns a sequence of vectors of dimension 32
model.add(LSTM(200))
#  model.add(Dense(10, activation='relu'))
model.add(Dense(3)) #, activation='sigmoid'))
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])
print(model.summary())

model.fit(train_x, train_y, epochs=10, batch_size=64, shuffle=True)

raw = pred_data.get_raw()

# 0-4 first one
res = model.predict(train_x)

df_res = pred_data.nn_to_data(res)

#  delay_pred = raw["x"]
#  delay_pred = delay_pred[6:-1].reset_index(drop=True)
actual = raw["x"]
delayed = actual.shift(6)

#  print("MSE", mean_squared_error(actual, delayed))
#  print("MSE", mean_squared_error(actual, delayed))

plt.plot(actual) # Our delayed version of the position
plt.plot(delayed) # Non-shifted, ie delayed

# at t = 10, we used t=5,6,7,8,9 to predict actual pose at t=10, which is at t=16 in vision
plt.plot(df_res["x"].shift(f_delay+batch_size)) # 6 frame ago predicted positions

# 0     1     2     3     4     5     6      7      8      9      10     11
# real0 real1 real2 real3 real4 real5 
#                                     delay0 delay1 delay2 delay3 delay4 delay5
#                                                                        predict11

plt.legend(["Actual", "Delayed", "Predicted"])

plt.show()

