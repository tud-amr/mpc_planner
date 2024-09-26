import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

import copy
import casadi as cd
import numpy as np

from control_modules import ObjectiveModule, Objective

from spline import Spline, Spline2D
from util.math import haar_difference_without_abs
from util.logging import print_warning

### MPCC

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

        # Static
        # reference_velocity = params.get("reference_velocity")

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


class CurvatureAwareContouringModule(ObjectiveModule):

    def __init__(self, settings):
        super().__init__()
        self.module_name = "Contouring"  # Needs to correspond to the c++ name of the module
        self.import_name = "contouring.h"
        self.type = "objective"


        self.description = "Curvature-Aware MPC: Tracks a 2D reference path with curvature-aware contouring costs"

        self.objectives = []
        self.objectives.append(CurvatureAwareContouringObjective(settings))


class ContouringModule(ObjectiveModule):

    def __init__(self, settings):#, num_segments, preview=0.0, use_ca_mpc=True):
        super().__init__()
        self.module_name = "Contouring"  # Needs to correspond to the c++ name of the module
        self.import_name = "contouring.h"
        self.type = "objective"

        self.description = "MPCC: Tracks a 2D reference path with contouring costs"

        self.objectives = []
        self.objectives.append(ContouringObjective(settings))




        #     def get_value(self, model, params, settings, stage_idx):
        # cost = 0

        # pos_x = model.get("x")
        # pos_y = model.get("y")
        # psi = model.get("psi")
        # v = model.get("v")
        # s = model.get("spline")

        # contour_weight = params.get("contour")
        # velocity_weight = params.get("velocity")

        # # From path
        # if self.dynamic_velocity_reference:
        #     if not params.has_parameter("spline_v0_a"):
        #         raise IOError("contouring/dynamic_velocity_reference is enabled, but there is no PathReferenceVelocity module.")
        
        #     path_velocity = Spline(params, "spline_v", self.num_segments, s)
        #     reference_velocity = path_velocity.at(s)

        # # Static
        # # reference_velocity = params.get("reference_velocity")

        # path = Spline2D(params, self.num_segments, s)
        # path_x, path_y = path.at(s)
        # path_dx_normalized, path_dy_normalized = path.deriv_normalized(s)

        # contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)

        # cost += contour_weight * contour_error**2

        # if not self.use_ca_mpc:
        #     # MPCC
        #     lag_weight = params.get("lag")

        #     contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
        #     lag_error = path_dx_normalized * (pos_x - path_x) + path_dy_normalized * (pos_y - path_y)

        #     cost += lag_weight * lag_error**2
        #     cost += velocity_weight * (v - reference_velocity) ** 2
        #     cost += contour_weight * contour_error**2
        # else:
        #     # CA-MPC
        #     # https://www.researchgate.net/profile/Laura-Ferranti-4/publication/371169207_Curvature-Aware_Model_Predictive_Contouring_Control/links/64775cecd702370600c50752/Curvature-Aware-Model-Predictive-Contouring-Control.pdf

        #     # dt = settings["integrator_step"]

        #     # vel = np.array([v * cd.cos(psi), v * cd.sin(psi)]) # Velocity vector

        #     # t_vec = np.array([path_dx_normalized, path_dy_normalized])
        #     # n_vec = np.array([path_dy_normalized, -path_dx_normalized])

        #     # vt = vel.dot(t_vec) # Velocity path components
        #     # vn = vel.dot(n_vec)

        #     # vt_t = vt * dt # Euler integrated velocities
        #     # vn_t = vn * dt

        #     # R = 1. / path.get_curvature(s) # max(R) = 1 / 0.0001
        #     # R = cd.fmax(R, 1e5)

        #     # # Path velocity
        #     # s_dot = R * vt * ((R - contour_error - vn_t) + vn_t) / ((R - contour_error - vn_t)**2 + (vt_t)**2)

        #     # Lorenzo's equations
        #     path_ddx, path_ddy = path.deriv2(s)
        #     projection_ratio = 1.0 / (1.0 - ((pos_x - path_x) * path_ddx + (pos_y - path_y) * path_ddy))
        #     s_dot = v * (cd.cos(psi) * path_dx_normalized + cd.sin(psi) * path_dy_normalized) * projection_ratio
        #     # Path projection is unnecessary with CA-MPC
        #     contour_error_squared = (pos_x - path_x) ** 2 + (pos_y - path_y) ** 2

        #     cost += contour_weight * contour_error_squared
        #     cost += velocity_weight * (s_dot - reference_velocity) ** 2  # Penalize its tracking performance

        # # Terminal cost
        # if True and stage_idx == settings["N"] - 1:

        #     terminal_angle_weight = params.get("terminal_angle")
        #     terminal_contouring_mp = params.get("terminal_contouring")

        #     # Compute the angle w.r.t. the path
        #     path_angle = cd.atan2(path_dy_normalized, path_dx_normalized)
        #     angle_error = haar_difference_without_abs(psi, path_angle)

        #     # Penalize the angle error
        #     cost += terminal_angle_weight * angle_error**2

        #     if not self.use_ca_mpc:
        #         cost += terminal_contouring_mp * lag_weight * lag_error**2
        #         cost += terminal_contouring_mp * contour_weight * contour_error**2
        #     else:
        #         cost += terminal_contouring_mp * contour_weight * contour_error_squared
        #         cost += terminal_contouring_mp * velocity_weight * (s_dot - reference_velocity) ** 2

        # return cost