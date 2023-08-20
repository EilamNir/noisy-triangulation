import numpy as np
import math

def transpose_matrix3d(phi, theta):
    return np.array([[math.cos(np.transpose(phi))*abs(math.sin(np.transpose(theta)))], [math.sin(np.transpose(phi))*abs(math.sin(np.transpose(theta)))], [math.cos(np.transpose(theta))]])

def transpose_matrix2d(phi):
    return np.array([[math.cos(np.transpose(phi))], [math.sin(np.transpose(phi))]])
