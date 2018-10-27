# Estimated ball postions using the kalman filter (For a single camera)
from ball.camera_ball import CameraBall
from filter.kalman_filter_2d import KalmanFilter2D

import util.config
import numpy as np
import matplotlib.pyplot as plt
import math

class KalmanBall:
    def __init__(self, time, camera_ball, previous_world_ball):
        init_pos = camera_ball.pos
        init_vel = [0, 0]
        if previous_world_ball is not None:
            init_vel = previous_world_ball.vel

        self.last_update_time = time
        self.last_predict_time = time
        self.filter = KalmanFilter2D(init_pos, init_vel)
        self.health = util.config.health_init

        # Previous raw measurements
        # Loop time the measurement occurred and the ball we used
        self.previous_measurement = [(time, camera_ball)]

        # Plotting stuff
        #self.setup_plots()

    def predict(self, time):
        self.last_predict_time = time
        self.health = max(self.health - util.config.health_dec, util.config.health_min)
        self.filter.predict()

        #self.plot_speed()

    def predict_and_update(self, time, camera_ball):
        self.last_update_time = time
        self.last_predict_time = time
        self.health = min(self.health + util.config.health_inc, util.config.health_max)

        self.filter.z_k = [[camera_ball.pos[0]], [camera_ball.pos[1]]]
        self.filter.predict_and_update()

        # Keep track of last X measurements
        self.previous_measurement.append((time, camera_ball))

        if len(self.previous_measurement) > util.config.slow_kick_detector_history_length:
            self.previous_measurement.pop(0)

        #self.plot_speed()

    def is_unhealthy(self):
        # Checks time since last update
        updated_recently = abs(self.last_update_time - self.last_predict_time) < util.config.ball_max_time_outside_vision

        return not updated_recently

    def pos(self):
        return [self.filter.x_k_k.item(0), self.filter.x_k_k.item(2)]

    def vel(self):
        return [self.filter.x_k_k.item(1), self.filter.x_k_k.item(3)]

    def set_vel(self, new_vel):
        self.filter.x_k_k[1] = new_vel[0]
        self.filter.x_k_k[3] = new_vel[1]

    def setup_plots(self):
        self.figure, self.ax = plt.subplots()
        self.speed_x, self.speed_y = self.ax.plot([],[], 'r', [],[], 'b')
        self.ax.axis([1, 100, -30, 30])
        self.speed_x.set_label('x vel')
        self.speed_y.set_label('y vel')
        self.ax.legend()
        self.figure.show()

        self.speed_x_list = [0]
        self.speed_y_list = [0]
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