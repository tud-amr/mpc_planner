import numpy as np
import casadi as cd


def rotation_matrix(angle):
    return np.array([[cd.cos(angle), -cd.sin(angle)],
                    [cd.sin(angle), cd.cos(angle)]])
