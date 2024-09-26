import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

import copy
import casadi as cd
import numpy as np

from control_modules import ObjectiveModule, Objective

from spline import Spline, Spline2D
from util.math import haar_difference_without_abs
from util.logging import print_warning

class ContouringObjective:

    def __init__(self, settings):

        self.num_segments = settings["contouring"]["num_segments"]
        self.dynamic_velocity_reference = settings["contouring"]["dynamic_velocity_reference"]

    def define_parameters(self, params):
        params.add("contour", add_to_rqt_reconfigure=True)
        params.add("lag", add_to_rqt_reconfigure=True)
        
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
        lag_weight = params.get("lag")

        # From path
        if self.dynamic_velocity_reference:
            if not params.has_parameter("spline_v0_a"):
                raise IOError("contouring/dynamic_velocity_reference is enabled, but there is no PathReferenceVelocity module.")
        
            path_velocity = Spline(params, "spline_v", self.num_segments, s)
            reference_velocity = path_velocity.at(s)
            velocity_weight = params.get("velocity")

        path = Spline2D(params, self.num_segments, s)
        path_x, path_y = path.at(s)
        path_dx_normalized, path_dy_normalized = path.deriv_normalized(s)

        # MPCC
        contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
        lag_error = path_dx_normalized * (pos_x - path_x) + path_dy_normalized * (pos_y - path_y)

        cost += lag_weight * lag_error**2
        cost += contour_weight * contour_error**2

        if self.dynamic_velocity_reference:
            cost += velocity_weight * (v - reference_velocity) ** 2

        # Terminal cost
        if True and stage_idx == settings["N"] - 1:

            terminal_angle_weight = params.get("terminal_angle")
            terminal_contouring_mp = params.get("terminal_contouring")

            # Compute the angle w.r.t. the path
            path_angle = cd.atan2(path_dy_normalized, path_dx_normalized)
            angle_error = haar_difference_without_abs(psi, path_angle)

            # Penalize the angle error
            cost += terminal_angle_weight * angle_error**2
            cost += terminal_contouring_mp * lag_weight * lag_error**2
            cost += terminal_contouring_mp * contour_weight * contour_error**2
           
        return cost


class ContouringModule(ObjectiveModule):

    def __init__(self, settings):
        super().__init__()
        self.module_name = "Contouring"  # Needs to correspond to the c++ name of the module
        self.import_name = "contouring.h"
        self.type = "objective"

        self.description = "MPCC: Tracks a 2D reference path with contouring costs"

        self.objectives = []
        self.objectives.append(ContouringObjective(settings))