import numpy as np

def rotation_matrix(phi):
    """
    Calculate a rotation matrix for the given angle.
    """
    return np.asmatrix([
        [np.cos(phi), -np.sin(phi), 0],
        [np.sin(phi), np.cos(phi), 0],
        [0, 0, 1]
    ])
