# Filtered robot
from filter.kalman_filter_3d import KalmanFilter3D

import util.config

class KalmanRobot:
    def __init__(self, confidence, x, y, orientation, bot_id):
        self.bot_id = bot_id

        self.filter = KalmanFilter3D(x, y, orientation)
        self.health = util.config.health_init
    
    def predict(self):
        self.health -= util.config.health_dec

    def predict_and_update(self, observation):
        self.health = max(self.health + util.config.health_inc, util.config.health_max)