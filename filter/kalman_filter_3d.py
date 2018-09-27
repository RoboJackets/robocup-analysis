# xy pos/vel and heading/angular velocity
from filter.kalman_filter import KalmanFilter
import util.config
import numpy as np

class KalmanFilter3D(KalmanFilter):
    def __init__(self, pos, vel):
        KalmanFilter.__init__(self, 6, 3)

        # States are X pos, X vel, Y pos, Y vel, heading, angular velocity
        # Assume 0 velocities
        self.x_k1_k1 = np.matrix([[pos[0]],
                                  [vel[0]],
                                  [pos[1]],
                                  [vel[1]],
                                  [pos[2]],
                                  [vel[2]]])

        # State transition matrix (A)
        # Pos, velocity, orientation integrator. Assume constant velocity
        dt = util.config.dt
        self.F_k = np.matrix([[1, dt, 0,  0, 0,  0],
                              [0,  1, 0,  0, 0,  0],
                              [0,  0, 1, dt, 0,  0],
                              [0,  0, 0,  1, 0,  0],
                              [0,  0, 0,  0, 1, dt],
                              [0,  0, 0,  0, 0,  1]])

        # Control transition matrix (B)
        # No inputs. This can change for our robots since we command accelerations
        self.B_k = np.matrix([[0],
                              [0],
                              [0],
                              [0],
                              [0],
                              [0]])

        # Observation matrix (C)
        # We can get the positions and heading
        self.H_k = np.matrix([[1, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0]])

        # Covariance of process noise (how wrong A is)
        # There is a constant deceleration so velocity should have a higher process noise
        # There aren't any major problems with the position process
        # May need to split the process noise between xy states and heading states
        p = util.config.robot_process_noise
        self.Q_k = np.matrix([[0, 0, 0, 0, 0, 0],
                              [0, p, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0],
                              [0, 0, 0, p, 0, 0],
                              [0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, p]])

        # Covariance of observation noise (how wrong z_k is)
        # May need to split observation noise between the xy states and heading states
        o = util.config.robot_observation_noise
        self.R_k = np.matrix([[o, 0, 0],
                              [0, o, 0],
                              [0, 0, o]])