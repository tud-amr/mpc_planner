import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

import copy
import casadi as cd
import numpy as np

from control_modules import ObjectiveModule, Objective

from spline import Spline, Spline2D
from util.math import haar_difference_without_abs
from util.logging import print_warning

class CurvatureAwareContouringObjective:

    def __init__(self, settings):

        self.num_segments = settings["contouring"]["num_segments"]
        self.dynamic_velocity_reference = settings["contouring"]["dynamic_velocity_reference"]

    def define_parameters(self, params):
        params.add("contour", add_to_rqt_reconfigure=True)
        params.add("lag", add_to_rqt_reconfigure=True) # Not used, but necessary to compile contouring c++ code
        
        if not params.has_parameter("velocity"):
            params.add("velocity", add_to_rqt_reconfigure=True)
            params.add("reference_velocity", add_to_rqt_reconfigure=True)

        params.add("terminal_angle", add_to_rqt_reconfigure=True)
        params.add("terminal_contouring", add_to_rqt_reconfigure=True)

        for i in range(self.num_segments):
            params.add(f"spline_x{i}_a", bundle_name="spline_x_a")
            params.add(f"spline_x{i}_b", bundle_name="spline_x_b")
            params.add(f"spline_x{i}_c", bundle_name="spline_x_c")
            params.add(f"spline_x{i}_d", bundle_name="spline_x_d")

            params.add(f"spline_y{i}_a", bundle_name="spline_y_a")
            params.add(f"spline_y{i}_b", bundle_name="spline_y_b")
            params.add(f"spline_y{i}_c", bundle_name="spline_y_c")
            params.add(f"spline_y{i}_d", bundle_name="spline_y_d")

            params.add(f"spline{i}_start", bundle_name="spline_start")

        return params

    def get_value(self, model, params, settings, stage_idx):
        cost = 0

        pos_x = model.get("x")
        pos_y = model.get("y")
        psi = model.get("psi")
        v = model.get("v")
        s = model.get("spline")

        contour_weight = params.get("contour")
        velocity_weight = params.get("velocity")

        # From path
        if self.dynamic_velocity_reference:
            if not params.has_parameter("spline_v0_a"):
                raise IOError("contouring/dynamic_velocity_reference is enabled, but there is no PathReferenceVelocity module.")
        
            path_velocity = Spline(params, "spline_v", self.num_segments, s)
            reference_velocity = path_velocity.at(s)
        else:
            reference_velocity = params.get("reference_velocity")

        path = Spline2D(params, self.num_segments, s)
        path_x, path_y = path.at(s)
        path_dx_normalized, path_dy_normalized = path.deriv_normalized(s)

        contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)

        # CA-MPC
        # https://www.researchgate.net/profile/Laura-Ferranti-4/publication/371169207_Curvature-Aware_Model_Predictive_Contouring_Control/links/64775cecd702370600c50752/Curvature-Aware-Model-Predictive-Contouring-Control.pdf

        # Lorenzo's equations
        path_ddx, path_ddy = path.deriv2(s)
        projection_ratio = 1.0 / (1.0 - ((pos_x - path_x) * path_ddx + (pos_y - path_y) * path_ddy))
        s_dot = v * (cd.cos(psi) * path_dx_normalized + cd.sin(psi) * path_dy_normalized) * projection_ratio
        
        # Path projection is not necessary with CA-MPC
        contour_error_squared = (pos_x - path_x) ** 2 + (pos_y - path_y) ** 2

        cost += contour_weight * contour_error_squared
        cost += velocity_weight * (s_dot - reference_velocity) ** 2  # Penalize its tracking performance

        # Terminal cost
        if True and stage_idx == settings["N"] - 1:

            terminal_angle_weight = params.get("terminal_angle")
            terminal_contouring_mp = params.get("terminal_contouring")

            # Compute the angle w.r.t. the path
            path_angle = cd.atan2(path_dy_normalized, path_dx_normalized)
            angle_error = haar_difference_without_abs(psi, path_angle)

            # Penalize the angle error
            cost += terminal_angle_weight * angle_error**2
            cost += terminal_contouring_mp * contour_weight * contour_error_squared
            cost += terminal_contouring_mp * velocity_weight * (s_dot - reference_velocity) ** 2

        return cost

class CurvatureAwareContouringModule(ObjectiveModule):

    def __init__(self, settings):
        super().__init__()

        self.module_name = "CurvatureAwareContouring"  # Needs to correspond to the c++ name of the module
        self.import_name = "curvature_aware_contouring.h"
        self.type = "objective"

        self.description = "Curvature-Aware MPC: Tracks a 2D reference path with curvature-aware contouring costs"

        self.objectives = []
        self.objectives.append(CurvatureAwareContouringObjective(settings))

        self.sources.append("contouring.cpp") # The C++ code uses the regular contouring code too

        print_warning("Using curvature-aware contouring, make sure that the model updates the spline parameter accordingly!")