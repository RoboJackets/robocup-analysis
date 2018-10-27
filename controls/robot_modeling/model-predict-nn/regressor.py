#!/bin/env python3
import pandas
from matplotlib import pyplot as plt
from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline

import pred_data

num_frames = 30;
num_inputs_per_frame = 7
input_size = num_frames * num_inputs_per_frame

f_delay = 6

(x, y) = pred_data.get_data_2d(frame_delay=f_delay, window_size=num_frames)

model = Sequential()
model.add(Dense(input_size, input_dim=input_size, kernel_initializer='normal', activation='relu'))
model.add(Dense(500, kernel_initializer='normal', activation='relu'))
model.add(Dense(500, kernel_initializer='normal', activation='relu'))
model.add(Dense(3, kernel_initializer='normal'))
# Compile model
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])

model.fit(x, y, epochs=200, batch_size=2, shuffle=True)

raw = pred_data.get_raw()

# 0-4 first one
res = model.predict(x)

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
plt.plot(df_res["x"].shift(f_delay+num_frames)) # 6 frame ago predicted positions

# 0     1     2     3     4     5     6      7      8      9      10     11
# real0 real1 real2 real3 real4 real5 
#                                     delay0 delay1 delay2 delay3 delay4 delay5
#                                                                        predict11

plt.legend(["Actual", "Delayed", "Predicted"])

plt.show()

