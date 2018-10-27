#!/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def log(*args, **kwargs):
    verbose = False
    if verbose:
        print("".join(map(str,args)), **kwargs)

def get_raw():
    data = pd.read_csv("../vision-enc-data/enc_vis.txt", sep=" ")
    # header: bot_id x y ang enc0 enc1 enc2 enc3
    data = data.drop("bot_id", axis=1)

    return data

def data_to_nn(df):
    pos_headers = ["x", "y"]
    enc_headers = ["enc0", "enc1", "enc2", "enc3"]

    df["ang"] = np.arctan2(np.sin(df["ang"]), np.cos(df["ang"])) / (2*np.pi)

    for h in pos_headers:
        df[h] = df[h] / 10000

    for h in enc_headers:
        df[h] = df[h] / 5000

    return df

def nn_to_data(np_array):
    df = pd.DataFrame(np_array, columns=["x","y","ang"])

    pos_headers = ["x", "y"]

    for h in pos_headers:
        df[h] = df[h] * 10000

    #  df["ang"] = df["ang"] * 2 * np.pi

    return df

def get_data_2d(frame_delay=6, window_size=3):
    (x_data, y_data) = get_data(frame_delay, window_size)

    new_x_data = []

    for input_list in x_data:
        new_x_data.append(input_list.as_matrix().flatten())

    return np.array(new_x_data), np.array(y_data)

def get_data(frame_delay=6, window_size=3):
    data = pd.read_csv("../vision-enc-data/enc_vis.txt", sep=" ")
    # header: bot_id x y ang enc0 enc1 enc2 enc3
    data = data.drop("bot_id", axis=1)

    data = data_to_nn(data)

    x_data = []
    y_data = []

    y_slice = np.split(data, [3], axis=1)[0]

    # we receive measurement at beginning of each frame

    # can only start predictions once window_size has finished
    for time_ind in range(0+window_size-1, len(data) - frame_delay):
        log("--------------------------------------------------------------------------------")
        #  log("Processing row {}".format(time_ind))

        # we have received all measurements up to and including time_ind
        x = data.iloc[time_ind-window_size+1:time_ind+1]
        # we just received time_ind, which is a frame_delay version of
        # current position. We estimate frame delay to be 6 cycles,
        # so take our time_ind and get 6 frames ahead.
        y = y_slice.iloc[time_ind + frame_delay]

        x_data.append(x)
        y_data.append(y)
        
        log("X data")
        log(x)
        log("Y_data")
        log(y)
        log("--------------------------------------------------------------------------------")
        log()

    return x_data, y_data
