import numpy as np
import casadi as cd

class SplineSegment:

    def __init__(self, param, name, spline_nr):

        # Retrieve spline values from the parameters (stored as multi parameter by name)
        self.a = param.get(f"{name}{spline_nr}_a")
        self.b = param.get(f"{name}{spline_nr}_b")
        self.c = param.get(f"{name}{spline_nr}_c")
        self.d = param.get(f"{name}{spline_nr}_d")

        self.s_start = param.get(f"spline{spline_nr}_start")

    def at(self, spline_index):
        s = spline_index - self.s_start
        return self.a * s * s * s + self.b * s * s + self.c * s + self.d

    def deriv(self, spline_index):
        s = spline_index - self.s_start
        return 3 * self.a * s * s + 2 * self.b * s + self.c
    
    def deriv2(self, spline_index):
        s = spline_index - self.s_start
        return 6 * self.a * s + 2 * self.b

class Spline:
    def __init__(self, params, name, num_segments, s):
        self.splines = []  # Classes containing the splines
        self.lambdas = []  # Merges splines
        for i in range(num_segments):
            self.splines.append(SplineSegment(params, f"{name}", i))

            # No lambda for the first segment (it is not glued to anything prior)
            if i > 0:
                self.lambdas.append(1.0 / (1.0 + np.exp((s - self.splines[-1].s_start + 0.02) / 0.1)))  # Sigmoid

    def at(self, s):
        # Iteratively glue segments together
        value = self.splines[-1].at(s)
        for k in range(len(self.splines) - 1, 0, -1):
            value = self.lambdas[k - 1] * self.splines[k - 1].at(s) + (1.0 - self.lambdas[k - 1]) * value
        return value
    
    def deriv(self, s):
        value = self.splines[-1].deriv(s)
        for k in range(len(self.splines) - 1, 0, -1):
            value = self.lambdas[k - 1] * self.splines[k - 1].deriv(s) + (1.0 - self.lambdas[k - 1]) * value
        return value
    
    def deriv2(self, s):
        value = self.splines[-1].deriv2(s)
        for k in range(len(self.splines) - 1, 0, -1):
            value = self.lambdas[k - 1] * self.splines[k - 1].deriv2(s) + (1.0 - self.lambdas[k - 1]) * value
        return value



class Spline2D:

    def __init__(self, params, num_segments, s):
        self.spline_x = Spline(params, "spline_x", num_segments, s)
        self.spline_y = Spline(params, "spline_y", num_segments, s)

    def at(self, s):
        return self.spline_x.at(s), self.spline_y.at(s)

    def deriv(self, s):
        return self.spline_x.deriv(s), self.spline_y.deriv(s)
    
    def deriv_normalized(self, s):
        dx = self.spline_x.deriv(s)
        dy = self.spline_y.deriv(s)
        path_norm = cd.sqrt(dx * dx + dy * dy)

        return dx / path_norm, dy / path_norm
    
    def deriv2(self, s):
        return self.spline_x.deriv2(s), self.spline_y.deriv2(s)

    def get_curvature(self, s):

        path_x_deriv2 = self.spline_x.deriv2(s)
        path_y_deriv2 = self.spline_y.deriv2(s)

        return cd.sqrt(path_x_deriv2 * path_x_deriv2 + path_y_deriv2 * path_y_deriv2)# + 0.0000000001) # Max = 1e2