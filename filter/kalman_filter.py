import numpy as np

class KalmanFilter:
    def __init__(self, state_size, observation_size):
        # x_[at time step]_[using data from time step]
        # k1 is k - 1
        self.x_k1_k1 = np.zeros((state_size, 1))
        self.x_k_k1  = np.zeros((state_size, 1))
        self.x_k_k   = np.zeros((state_size, 1))

        self.u_k    = np.zeros((1, 1))
        self.z_k    = np.zeros((state_size, 1))

        self.y_k_k1 = np.zeros((observation_size, 1))
        self.y_k_k  = np.zeros((observation_size, 1))

        self.P_k1_k1 = np.identity((state_size))
        self.P_k_k1  = np.identity((state_size))
        self.P_k_k   = np.identity((state_size))

        self.S_k = np.zeros((observation_size, observation_size))
        self.K_k = np.zeros((state_size, observation_size))

        self.F_k = np.zeros((state_size, state_size))
        self.B_k = np.zeros((state_size, 1))
        self.H_k = np.zeros((observation_size, state_size))
        self.Q_k = np.zeros((state_size, state_size))
        self.R_k = np.zeros((observation_size, observation_size))

        self.I = np.identity(state_size)

    def predict(self):
        self.x_k1_k1 = self.x_k_k
        self.P_k1_k1 = self.P_k_k

        self.x_k_k1 = self.F_k * self.x_k1_k1 + self.B_k*self.u_k
        self.P_k_k1 = self.F_k * self.P_k1_k1 * self.F_k.transpose() + self.Q_k

        self.x_k_k = self.x_k_k1
        self.P_k_k = self.P_k_k1
            
    def predict_and_update(self):
        self.x_k1_k1 = self.x_k_k
        self.P_k1_k1 = self.P_k_k

        self.x_k_k1 = self.F_k * self.x_k1_k1 + self.B_k*self.u_k
        
        self.P_k_k1 = self.F_k * self.P_k1_k1 * self.F_k.transpose() + self.Q_k

        self.y_k_k1 = self.z_k - self.H_k * self.x_k_k1
        
        self.S_k = self.R_k + self.H_k * self.P_k_k1 * self.H_k.transpose()

        self.K_k = self.P_k_k1 * self.H_k.transpose() * self.S_k.getI()

        self.x_k_k = self.x_k_k1 + self.K_k * self.y_k_k1

        self.P_k_k = (self.I - self.K_k * self.H_k) * self.P_k_k1 * (self.I - self.K_k * self.H_k).transpose() + self.K_k * self.R_k * self.K_k.transpose()

        self.y_k_k = self.z_k - self.H_k * self.x_k_k