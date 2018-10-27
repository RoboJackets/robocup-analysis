#!/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from matplotlib import pyplot as plt
from matplotlib import animation

data = pd.read_csv("../vision-enc-data/enc_vis.txt", sep=" ")
data = data.drop("bot_id", axis=1)

# header: bot_id x y ang enc0 enc1 enc2 enc3


for index, row in data.iterrows():
    break
    #  plt.plot(row["x"], row["y"])

#  plt.plot(data["x"], data["y"])

#  plt.plot(data)
#  plt.show()

#  print(data)

#  exit(0)

fig = plt.figure()
ax = plt.axes(xlim=(min(data['x']), max(data['x'])), ylim=(min(data['y']),max(data['y'])))
#  ax = plt.axes(xlim=(0,2), ylim=(-2,2))
line, = ax.plot([], [], lw=2)

def init():
    line.set_data([], [])
    return line,

def animate(i):
    #  print(i)
    #  row = data.iloc[i]
    #  x = row['x']
    #  y = row['y']
    #  x = 1000*np.linspace(0, 2, 1000)
    #  y = 1000*np.sin(2 * np.pi * (x - 0.01 * i))
    st = 1000
    x = data.iloc[st:st+i]['x']
    y = data.iloc[st:st+i]['y']
    #  print(x)
    #  line.set_data(data.iloc[0:i]['x'], data.iloc[0:i]['y'])
    line.set_data(x, y)
    #  print(len(line.get_xdata()))
    #  line.set_data(data['x'], data['y'])
    return line,

print(len(data))

anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=len(data), interval=1000*1/60.0, blit=True)

plt.show()


#  def get_income(one_hot, fill_nan, drop_cat):
    #  income = pd.read_csv("income_dataset/train_students.txt",
                          #  names=["Id", "age", "workclass", "fnlwgt", "education",
                                #  "edu-num", "marital-status", "occupation",
                                #  "relationship", "race", "sex", "capital-gain",
                                #  "capital-loss", "hours-per-week", "native-country",
                                #  "Prediction"],
                          #  sep=" *, *",
                          #  engine="python")
    #  income = income.iloc[1:]

    #  # remove rows with missing data
    #  income = income.applymap(lambda x: np.nan if x.strip() == "?" else x)
    #  if not fill_nan:
        #  income = income.dropna()
    #  # education is already numerically encoded
    #  income = income.drop("education", axis=1)
    #  income = income.drop("Id", axis=1)
    #  income = income.drop("fnlwgt", axis=1)
    #  #income = income.drop("native-country", axis=1)
    #  #income = income[["marital-status", "relationship","race","Prediction"]]
    #  #income = income.drop("marital-status", axis=1)
    #  #income = income.drop("relationship", axis=1)

    #  str_cols = []
    #  num_cols = []
    #  for col_name in income.columns:
        #  income[col_name] = pd.to_numeric(income[col_name], errors="ignore")
        #  if (type(income[col_name].iloc[0]) is str):
            #  str_cols.append(col_name)
            #  if fill_nan:
                #  income[col_name] = income[col_name].fillna(income[col_name].value_counts().idxmax())
            #  if not one_hot:
                #  income[col_name] = income[col_name].astype("category").cat.codes
        #  else:
            #  num_cols.append(col_name)
            #  if fill_nan:
                #  income[col_name] = income[col_name].fillna(income[col_name].mean())

    #  y_name = "Prediction"
    #  income_y = income[y_name]
    #  income = income.drop(y_name, axis=1)
    
    #  if drop_cat:
        #  print("killed: ", str_cols)
        #  income_X = income.drop(str_cols, axis=1)
    #  elif one_hot:
        #  income_X = pd.get_dummies(income, columns=str_cols)
    #  else:
        #  income_X = income

    #  income_X -= income_X.min()
    #  income_X /= income_X.max()
    #  income_X = income_X * 2 - 1

    #  # income data set, complete with data and labels in tuple
    #  income = (income_X, income_y)
    
    #  return income

#  def get_orig_income(index):
    #  income = pd.read_csv("income_dataset/train_students.txt",
                          #  names=["Id", "age", "workclass", "fnlwgt", "education",
                                #  "edu-num", "marital-status", "occupation",
                                #  "relationship", "race", "sex", "capital-gain",
                                #  "capital-loss", "hours-per-week", "native-country",
                                #  "Prediction"],
                          #  sep=" *, *",
                          #  engine="python")
    #  income = income.iloc[1:]
    #  return income.iloc[index]
    
#  def get_digits():
    #  from sklearn import datasets

    #  # Digits data set, complete with data and labels in tuple
    #  digits_X, digits_y = datasets.load_digits(return_X_y=True)
    #  digits_X -= digits_X.min()
    #  digits_X /= digits_X.max()
    #  digits_X = digits_X * 2 - 1

    #  digits = (pd.DataFrame(digits_X), pd.Series(digits_y))
    
    #  return digits
