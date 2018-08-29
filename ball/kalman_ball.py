# Estimated ball postions using the kalman filter (For a single camera)
from ball.camera_ball import CameraBall
from filter.kalman_filter_2d import KalmanFilter2D

import util.config

class KalmanBall:
    def __init__(self, initial_state):        
        self.filter = KalmanFilter2D(initial_state)
        self.health = util.config.health_init

    def predict(self):
        self.health -= util.config.health_dec

    def predict_and_update(self, observation):
        self.health = max(self.health + util.config.health_inc, util.config.health_max)