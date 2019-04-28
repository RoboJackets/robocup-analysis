import numpy as np
import util

class Controller:
    def __init__(self, dt, model):
        self.integral = np.asmatrix(np.zeros((3, 1)))
        self.dt = dt
        self.Kp, self.Ki, self.Kd = (50, 0, 5)
        self.model = model
        self.Mp = np.diag([50, 50, 100])
        self.Mi = 0 * np.diag([50, 50, 100])
        self.Md = np.diag([15, 15, 60])

    def reset(self):
        self.integral = np.asmatrix(np.zeros((3, 1)))

    def control(self, x, v, rx, rv, ra):
        gRb = util.rotation_matrix(x[2, 0])

        gRp = util.rotation_matrix(rx[2, 0])
        error_path_relative = gRp * (rx - x)

        self.integral += self.dt * error_path_relative

        G = self.model.geom

        GTinv = np.linalg.pinv(G.T * 0.029)
        error = gRb.T * (rx - x)
        error[2, 0] *= 0.1

        derivative = gRb.T * (rv - v)
        derivative[2, 0] *= 0.01
        goal_acceleration = ra + self.Mp * (rx - x) + self.Md * (rv - v) + self.Mi * gRp.T * self.integral
        return self.model.inverse_dynamics_world(x, rv, goal_acceleration)
        uff = self.model.inverse_dynamics_world(x, rv, ra)
        up = self.Kp * GTinv * error
        ui = self.Ki * GTinv * np.diag([1, 1, 0]) * gRb * gRp.T * self.integral
        ud = self.Kd * GTinv * derivative
        return uff + up + ui + ud
