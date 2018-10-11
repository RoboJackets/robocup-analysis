# xy pos/vel and heading/angular velocity
from filter.kalman_filter import KalmanFilter
import util.config
import numpy as np
import math

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
        self.x_k_k1 = self.x_k1_k1
        self.x_k_k = self.x_k1_k1

        # Initial covariance is usually extremely high to converge to the true solution
        p = util.config.ball_init_covariance
        self.P_k1_k1 = np.matrix([[p, 0, 0, 0, 0, 0],
                                  [0, p, 0, 0, 0, 0],
                                  [0, 0, p, 0, 0, 0],
                                  [0, 0, 0, p, 0, 0],
                                  [0, 0, 0, 0, p, 0],
                                  [0, 0, 0, 0, 0, p]])
        self.P_k_k1 = self.P_k1_k1
        self.P_k_k = self.P_k_k

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
        # Based on a guassian white noise w_k in x_dot = A*x + B*u + G*w
        # The noise can be propogated through the model resulting in a process noise
        # of the form
        #
        #  [1/3 T^3     1/2 T^2] * sigma^2
        #  [1/2 T^2           T]
        # Where sigma is the standard deviation of the process noise
        # the change in velocity over one time step should be around sqrt(T * sigma^2)
        # Note: T is the sample period
        # Note: May need second process noise specifically for the orientation
        p = util.config.robot_process_noise
        sigma = math.sqrt((3.0 * p) / dt) / dt
        dt3 = 1.0 / 3.0 * dt * dt * dt * sigma * sigma
        dt2 = 1.0 / 2.0 * dt * dt * sigma * sigma
        dt1 = dt * sigma * sigma

        self.Q_k = np.matrix([[dt3, dt2,   0,   0,   0,   0],
                              [dt2, dt1,   0,   0,   0,   0],
                              [0,     0, dt3, dt2,   0,   0],
                              [0,     0, dt2, dt1,   0,   0],
                              [0,     0,   0,   0, dt3, dt2],
                              [0,     0,   0,   0, dt2, dt1]])

        # Covariance of observation noise (how wrong z_k is)
        # May need to split observation noise between the xy states and heading states
        o = util.config.robot_observation_noise
        self.R_k = np.matrix([[o, 0, 0],
                              [0, o, 0],
                              [0, 0, o]])