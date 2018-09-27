# Filtered robot
from filter.kalman_filter_3d import KalmanFilter3D

import util.config

class KalmanRobot:
    def __init__(self, time, pos, vel, bot_id):
        self.last_update_time = time
        self.filter = KalmanFilter3D(pos, vel)
        self.health = util.config.health_init
        self.bot_id = bot_id
        self.unwrap_theta_ctr = 0
        self.previous_state = self.pos()

    def predict(self):
        self.health = max(self.health - util.config.health_dec, util.config.health_min)
        self.filter.predict()
        self.check_theta_unwrap()

    def predict_and_update(self, time, pos):
        self.last_update_time = time
        self.health = min(self.health + util.config.health_inc, util.config.health_max)

        # Unwrap yaw so we don't have any discontinuities
        pos[2] = pos[2] + self.unwrap_theta_ctr*2*3.14

        self.filter.z_k = [[pos[0]], [pos[1]], [pos[2]]]
        self.filter.predict_and_update()
        self.check_theta_unwrap()

    def pos(self):
        return [self.filter.x_k_k[0], self.filter.x_k_k[2], self.filter.x_k_k[4]]

    def is_unhealthy(self):
        # Checks how many frames it's has dropped recently
        valid_health = self.health <= util.config.health_bad
        # Checks time since last update (May not trigger a bad health though)
        updated_recently = True

        return valid_health and updated_recently

    def check_theta_unwrap(self):
        # Wrap below pi
        if (self.previous_state[2] < -3 and self.pos()[2] > 3):
            self.unwrap_theta_ctr -= 1

        # Wrap above pi
        if (self.previous_state[2] > 3 and self.pos()[2] < -3):
            self.unwrap_theta_ctr += 1

        self.previous_state = self.pos()