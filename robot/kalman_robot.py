# Filtered robot
from filter.kalman_filter_3d import KalmanFilter3D

import util.config

class KalmanRobot:
    def __init__(self, time, pos, vel, theta, omega, bot_id):
        self.last_update_time = time
        self.filter = KalmanFilter3D(pos, vel, theta, omega)
        self.health = util.config.health_init
        self.bot_id = bot_id

    def predict(self):
        self.health = max(self.health - util.config.health_dec, util.config.health_min)
        self.filter.predict()

    def predict_and_update(self, time, pos, theta):
        self.last_update_time = time
        self.health = min(self.health + util.config.health_inc, util.config.health_max)

        self.filter.z_k = [[pos], [theta]]
        self.filter.predict_and_update()

    def is_unhealthy(self):
        # Checks how many frames it's has dropped recently
        valid_health = self.health <= util.config.health_bad
        # Checks time since last update (May not trigger a bad health though)
        updated_recently = True

        return valid_health and updated_recently