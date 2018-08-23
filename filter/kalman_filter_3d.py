# xy pos/vel and heading/angular velocity
from filter.kalman_filter import KalmanFilter

class KalmanFilter3D(KalmanFilter):
    def __init__(self, initial_state, initial_covariance, process_noise, observation_noise):
        KalmanFilter.__init__(self, 6)

        # State transition matrix (A)
        # F_k
        # Control transition matrix (B)
        # B_k
        # Observation matrix (C)
        # H_k

        # Covariance of process noise (how wrong A is)
        # Q_k
        # Covariance of observation noise (how wrong z_k is)
        # R_k
        pass