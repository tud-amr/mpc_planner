import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-solver-generator"))

import casadi as cd
import numpy as np

from control_modules import ObjectiveModule, Objective


class MultiSplineXY:

    def __init__(self, params, num_segments, s):
        self.splines = []  # Classes containing the splines
        self.lambdas = []  # Merges splines
        for i in range(num_segments):
            self.splines.append(SplineXY(params, i))
            self.splines[-1].compute_point(s)

            # No lambda for the first segment (it is not glued to anything prior)
            if i > 0:
                self.lambdas.append(1.0 / (1.0 + np.exp((s - self.splines[-1].s_start + 0.02) / 0.1)))  # Sigmoid

    def get_path_and_dpath(self):
        # We iteratively glue paths together here (start with the last path)
        path_x = self.splines[-1].path_x
        path_y = self.splines[-1].path_y
        path_dx = self.splines[-1].path_dx
        path_dy = self.splines[-1].path_dy
        for k in range(len(self.splines) - 1, 0, -1):
            # Glue with the previous path
            path_x = self.lambdas[k - 1] * self.splines[k - 1].path_x + (1.0 - self.lambdas[k - 1]) * path_x
            path_y = self.lambdas[k - 1] * self.splines[k - 1].path_y + (1.0 - self.lambdas[k - 1]) * path_y
            path_dx = self.lambdas[k - 1] * self.splines[k - 1].path_dx + (1.0 - self.lambdas[k - 1]) * path_dx
            path_dy = self.lambdas[k - 1] * self.splines[k - 1].path_dy + (1.0 - self.lambdas[k - 1]) * path_dy

        path_norm = np.sqrt(path_dx**2 + path_dy**2)
        path_dx_normalized = path_dx / path_norm
        path_dy_normalized = path_dy / path_norm

        return path_x, path_y, path_dx_normalized, path_dy_normalized


class SplineXY:

    def __init__(self, param, spline_nr):

        # Retrieve spline values from the parameters (stored as multi parameter by name)
        self.x_a = param.get(f"spline{spline_nr}_ax")
        self.x_b = param.get(f"spline{spline_nr}_bx")
        self.x_c = param.get(f"spline{spline_nr}_cx")
        self.x_d = param.get(f"spline{spline_nr}_dx")

        self.y_a = param.get(f"spline{spline_nr}_ay")
        self.y_b = param.get(f"spline{spline_nr}_by")
        self.y_c = param.get(f"spline{spline_nr}_cy")
        self.y_d = param.get(f"spline{spline_nr}_dy")

        self.s_start = param.get(f"spline{spline_nr}_start")

    def compute_point(self, spline_index):
        self.path_x = (
            self.x_a * (spline_index - self.s_start) ** 3
            + self.x_b * (spline_index - self.s_start) ** 2
            + self.x_c * (spline_index - self.s_start)
            + self.x_d
        )

        self.path_y = (
            self.y_a * (spline_index - self.s_start) ** 3
            + self.y_b * (spline_index - self.s_start) ** 2
            + self.y_c * (spline_index - self.s_start)
            + self.y_d
        )

        self.path_dx = 3 * self.x_a * (spline_index - self.s_start) ** 2 + 2 * self.x_b * (spline_index - self.s_start) + self.x_c

        self.path_dy = 3 * self.y_a * (spline_index - self.s_start) ** 2 + 2 * self.y_b * (spline_index - self.s_start) + self.y_c


class ContouringObjective:

    def __init__(self, settings, num_segments):

        self.num_segments = num_segments

    def define_parameters(self, params):
        # Contour and lag are rqt parameters

        params.add("contour")
        params.add("lag")

        for i in range(self.num_segments):
            params.add(f"spline{i}_ax")
            params.add(f"spline{i}_bx")
            params.add(f"spline{i}_cx")
            params.add(f"spline{i}_dx")

            params.add(f"spline{i}_ay")
            params.add(f"spline{i}_by")
            params.add(f"spline{i}_cy")
            params.add(f"spline{i}_dy")

            params.add(f"spline{i}_start")

        return params

    def get_value(self, model, params, settings, stage_idx):
        cost = 0

        pos_x = model.get("x")
        pos_y = model.get("y")
        s = model.get("spline")

        contour_weight = params.get("contour")
        lag_weight = params.get("lag")

        splines = MultiSplineXY(params, self.num_segments, s)
        path_x, path_y, path_dx_normalized, path_dy_normalized = splines.get_path_and_dpath()

        contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
        lag_error = -path_dx_normalized * (pos_x - path_x) - path_dy_normalized * (pos_y - path_y)

        cost += contour_weight * contour_error**2
        cost += lag_weight * lag_error**2

        return cost


class ContouringModule(ObjectiveModule):

    def __init__(self, settings, num_segments):
        super().__init__()
        self.module_name = "Contouring"  # Needs to correspond to the c++ name of the module
        self.import_name = "contouring.h"
        self.type = "objective"
        self.description = "Tracks a 2D reference path with contouring costs"

        self.objectives = []
        self.objectives.append(ContouringObjective(settings, num_segments))
