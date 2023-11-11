import numpy as np


def get_rotate_x(x):
    c = np.cos(x)
    s = np.sin(x)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ])


def get_rotate_y(y):
    c = np.cos(y)
    s = np.sin(y)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ])


def get_rotate_z(z):
    c = np.cos(z)
    s = np.sin(z)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])
