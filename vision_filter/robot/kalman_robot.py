# Filtered robot
from filter.kalman_filter_3d import KalmanFilter3D

import util.config

class KalmanRobot:
    def __init__(self, time, pos, vel, bot_id):
        # Last time we had real data
        self.last_update_time = time
        # Last time time we predicted with/without data
        self.last_predict_time = time

        self.filter = KalmanFilter3D(pos, vel)
        self.health = util.config.health_init
        self.bot_id = bot_id

        # Unwrap theta stuff
        self.unwrap_theta_ctr = 0
        self.previous_measurement = pos[2]

        # History of all balls used to update
        self.camera_ball_history_list = []

    def predict(self, time):
        self.last_predict_time = time
        self.health = max(self.health - util.config.health_dec, util.config.health_min)
        self.filter.predict()

    def predict_and_update(self, time, pos):
        # Update health and access times
        self.last_update_time = time
        self.last_predict_time = time
        self.health = min(self.health + util.config.health_inc, util.config.health_max)

        # Unwrap yaw so we don't have any discontinuities
        self.check_theta_unwrap(time, pos)
        pos[2] = pos[2] + self.unwrap_theta_ctr*2*3.14

        # Apply filter
        self.filter.z_k = [[pos[0]], [pos[1]], [pos[2]]]
        self.filter.predict_and_update()

    def pos(self):
        return [self.filter.x_k_k[0], self.filter.x_k_k[2], self.filter.x_k_k[4]]

    def is_unhealthy(self):
        # Checks time since last update
        updated_recently = abs(self.last_update_time - self.last_predict_time) < util.config.robot_max_time_outside_vision

        return not updated_recently

    def check_theta_unwrap(self, time, pos):
        # We check below and above 2 just to give a little leyway between frames
        # Wrap below pi
        if (self.previous_measurement < -2 and pos[2] > 2):
            self.unwrap_theta_ctr -= 1

        # Wrap above pi
        if (self.previous_measurement > 2 and pos[2] < -2):
            self.unwrap_theta_ctr += 1

        self.previous_measurement = pos[2]