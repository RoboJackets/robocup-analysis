"""
Robocup robot model.

State:
[ x_world ] X position, in world coordinates.
[ y_world ] Y position, in world coordinates.
[   phi   ] Angle of the robot in world coordinates. When phi=0, x is to the
            right of the robot and y is forwards.
[ omega_1 ] Velocity of the first wheel
[ omega_2 ] Velocity of the first wheel
[ omega_3 ] Velocity of the first wheel
[ omega_4 ] Velocity of the first wheel
"""

import numpy as np
import math

# Robot mass, kg
m = 0.743

# Robot radius, m
L = 0.0789

# Gear ratio
n = 15.0 / 51.0

# Robot moment of inertia, kg*m^2
J = 0.5 * m * L ** 2

# Wheel radius, m
r = 0.029

# Resistance

# Wheel moment of inertia, kg*m^2
Jl = 2.4e-5
Jm = 135 * 1e-3 * (1e-2) ** 2

wheel_angles = [math.pi * 0.25,
                math.pi * 0.75,
                math.pi * 1.25,
                math.pi * 1.75]

# Geometry matrix
# Calculates wheel velocities from generalized velocities.
G = np.asmatrix([[-math.sin(th), math.cos(th), L] for th in wheel_angles]).T

# Drag coefficients
c_l = 0.001
c_m = 0

# Mass matrix
M = np.asmatrix([
    [m, 0, 0],
    [0, m, 0],
    [0, 0, J],
])


def rotation_matrix(phi):
    """
    Calculate a rotation matrix for the given angle.
    """
    return np.asmatrix([
        [math.cos(phi), -math.sin(phi), 0],
        [math.sin(phi), math.cos(phi), 0],
        [0, 0, 1]
    ])


def forward_dynamics(x, u, phi):
    """
    Run the continuous model to get dx/dt.

    x: world space velocities
    u: motor torques
    """
    phidot = x[2, 0]
    I4 = np.asmatrix(np.eye(4))
    GTinv = np.linalg.pinv(G.T)
    Ginv = np.linalg.pinv(G)
    gRb = rotation_matrix(phi)
    w_l = G.T * gRb.T * x / r

    Rdot = np.asmatrix([
        [-math.sin(phi), -math.cos(phi), 0],
        [math.cos(phi), -math.sin(phi), 0],
        [0, 0, 0]
    ]) * -phidot

    RTRdot = np.asmatrix([
        [0, 1, 0],
        [-1, 0, 0],
        [0, 0, 0]
    ]) * phidot

    Z = ((Jm + Jl / n ** 2) * I4
         + r ** 2 / n ** 2 * Ginv * M * GTinv)

    V = ((c_m + c_l / n ** 2) * I4
         + r ** 2 / n ** 2 * Ginv * RTRdot * M * GTinv)

    # T_m = Z*w'_m + Vw_m
    # w_m = w_l * n
    B = np.linalg.pinv(Z) / n
    A = -np.linalg.pinv(Z) * V
    wldot = A * w_l + B * u

    # X'' = gRb'inv(G.T)rw_l
    return Rdot * GTinv * r * w_l + gRb * GTinv * r * wldot

def inverse_dynamics(v, a, phi):
    """
    Run the continuous model to get u from dx/dt.

    v: world space velocities
    a: world space accelerations
    """
    phidot = v[2, 0]
    I4 = np.asmatrix(np.eye(4))
    GTinv = np.linalg.pinv(G.T)
    Ginv = np.linalg.pinv(G)
    gRb = rotation_matrix(phi)
    w_l = G.T * gRb.T * v / r

    Rdot = np.asmatrix([
        [-math.sin(phi), -math.cos(phi), 0],
        [math.cos(phi), -math.sin(phi), 0],
        [0, 0, 0]
    ]) * -phidot

    RTRdot = np.asmatrix([
        [0, 1, 0],
        [-1, 0, 0],
        [0, 0, 0]
    ]) * phidot

    Z = ((Jm + Jl / n ** 2) * I4
         + r ** 2 / n ** 2 * Ginv * M * GTinv)

    V = ((c_m + c_l / n ** 2) * I4
         + r ** 2 / n ** 2 * Ginv * RTRdot * M * GTinv)

    # Find wldot
    wldot = G.T * gRb.T / r * (a - Rdot * GTinv * r * w_l)

    # T_m = Z*w'_m + Vw_m
    # w_m = w_l / n
    # B = np.linalg.pinv(Z) * n
    A = -np.linalg.pinv(Z) * V
    # wldot = A * w_l + B * u
    u = Z * n * (wldot - A * w_l)

    return u

def main():
    """Run the program."""
    x = 15 * np.asmatrix([[1, -1, -1, 1]]).T
    u = np.asmatrix([[0.015, -0.015, -0.015, 0.015]]).T
    xdot = forward_dynamics(x, u)
    Ginv = np.linalg.pinv(G)


if __name__ == '__main__':
    main()
