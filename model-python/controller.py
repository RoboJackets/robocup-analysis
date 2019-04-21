import numpy as np
import math
import model

wheel_angles = [np.radians(30),
                np.radians(150),
                np.radians(-150),
                np.radians(-39)]

L = 0.0789

G = np.asmatrix([[-math.sin(th), math.cos(th), L] for th in wheel_angles]).T

class Controller:
    def __init__(self, dt):
        self.integral = np.asmatrix(np.zeros((3, 1)))
        self.dt = dt
        self.Kp, self.Ki, self.Kd = (5, 0.0, 10)

    def reset(self):
        self.integral = np.asmatrix(np.zeros((3, 1)))

    def control(self, x, v, rx, rv, ra):
        gRb = model.rotation_matrix(x[2])

        gRp = model.rotation_matrix(rx[2])
        error_path_relative = gRp * (rx - x)

        self.integral += self.dt * np.asmatrix(np.diag([1, 1, 1e-2])) \
            * error_path_relative

        Ginv = np.linalg.pinv(G)
        error = gRb.T * (rx - x)
        error[2, 0] *= 1e-2

        derivative = gRb.T * (rv - v)
        derivative[2, 0] *= 0.01

        uff = model.inverse_dynamics(rv, ra, x[2, 0])
        up = self.Kp * Ginv * error
        ui = self.Ki * Ginv * gRb * gRp.T * self.integral
        ud = self.Kd * Ginv * derivative
        return uff + up + ui + ud
