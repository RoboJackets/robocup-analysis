# Estimated ball postions using the kalman filter (For a single camera)
from ball.camera_ball import CameraBall
from filter.kalman_filter_2d import KalmanFilter2D

import util.config
import numpy as np
import matplotlib.pyplot as plt
import math

class KalmanBall:
    def __init__(self, time, pos, vel):
        self.last_update_time = time
        self.filter = KalmanFilter2D(pos, vel)
        self.health = util.config.health_init

        # Plotting stuff
        #self.setup_plots()

    def predict(self):
        self.health = max(self.health - util.config.health_dec, util.config.health_min)
        self.filter.predict()

        #self.plot_speed()

    def predict_and_update(self, time, pos):
        self.last_update_time = time
        self.health = min(self.health + util.config.health_inc, util.config.health_max)

        self.filter.z_k = [[pos[0]], [pos[1]]]
        self.filter.predict_and_update()

        #self.plot_speed()

    def is_unhealthy(self):
        # Checks how many frames it's has dropped recently
        valid_health = self.health <= util.config.health_bad
        # Checks time since last update (May not trigger a bad health though)
        updated_recently = True

        return valid_health and updated_recently

    def setup_plots(self):
        self.figure, self.ax = plt.subplots()
        self.speed_x, self.speed_y = self.ax.plot([],[], 'r', [],[], 'b')
        self.ax.axis([1, 100, -30, 30])
        self.speed_x.set_label('x vel')
        self.speed_y.set_label('y vel')
        self.ax.legend()
        self.figure.show()

        self.speed_x_list    = [0]
        self.speed_y_list    = [0]
        self.time = [0]

    def plot_speed(self):
        state = self.filter.x_k_k

        self.speed_x_list.append(state.item(1))
        self.speed_y_list.append(state.item(3))
        self.time.append(self.time[len(self.time) - 1] + 1)

        self.speed_x.set_xdata(self.time)
        self.speed_x.set_ydata(self.speed_x_list)

        self.speed_y.set_xdata(self.time)
        self.speed_y.set_ydata(self.speed_y_list)

        self.figure.canvas.draw()
        self.figure.canvas.flush_events()