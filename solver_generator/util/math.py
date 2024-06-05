import numpy as np
import casadi as cd


def rotation_matrix(angle):
    return np.array([[cd.cos(angle), -cd.sin(angle)],
                    [cd.sin(angle), cd.cos(angle)]])


def haar_difference_without_abs(angle1, angle2):
    return cd.fmod(angle1 - angle2 + np.pi, 2*np.pi) - np.pi