import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

import copy
import casadi as cd
import numpy as np

from control_modules import ObjectiveModule, Objective

from spline import Spline, Spline2D
from util.math import haar_difference_without_abs


def get_preview_state(model, settings, time_ahead):
    # Integrate the trajectory to obtain the preview point
    copied_model = copy.deepcopy(model)

    z_constant_input = cd.vertcat(np.zeros((model.nu)), model.get_x())

    z_preview = model.integrate(z_constant_input, settings, integration_step=time_ahead)  # Integrate the dynamics T seconds ahead
    z_preview = cd.vertcat(np.zeros((model.nu)), z_preview)

    # Retrieve the resulting position and spline
    copied_model.load(z_preview)
    pos_x = copied_model.get("x")
    pos_y = copied_model.get("y")
    s = copied_model.get("spline")

    return pos_x, pos_y, s


class ContouringObjective:

    def __init__(self, settings, num_segments, preview=0.0, use_ca_mpc=True):

        self.num_segments = num_segments

        self.enable_preview = preview != 0.0
        self.preview = preview

        self.use_ca_mpc = use_ca_mpc

    def define_parameters(self, params):
        params.add("contour", add_to_rqt_reconfigure=True)

        params.add("reference_velocity", add_to_rqt_reconfigure=True)
        params.add("velocity", add_to_rqt_reconfigure=True)
        params.add("lag", add_to_rqt_reconfigure=True)

        params.add("terminal_angle", add_to_rqt_reconfigure=True)
        params.add("terminal_contouring", add_to_rqt_reconfigure=True)

        if self.enable_preview:
            params.add("preview", add_to_rqt_reconfigure=True)

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
        path_velocity = Spline(params, "spline_v", self.num_segments, s)
        reference_velocity = path_velocity.at(s)

        # Static
        # reference_velocity = params.get("reference_velocity")

        path = Spline2D(params, self.num_segments, s)
        path_x, path_y = path.at(s)
        path_dx_normalized, path_dy_normalized = path.deriv_normalized(s)

        contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
        # contour_error_squared = (pos_x - path_x) **2 + (pos_y - path_y) ** 2

        cost += contour_weight * contour_error**2

        if not self.use_ca_mpc:
            # MPCC
            lag_weight = params.get("lag")

            contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
            lag_error = path_dx_normalized * (pos_x - path_x) + path_dy_normalized * (pos_y - path_y)

            cost += lag_weight * lag_error**2
            cost += velocity_weight * (v - reference_velocity) ** 2
            cost += contour_weight * contour_error**2
        else:
            # CA-MPC
            # https://www.researchgate.net/profile/Laura-Ferranti-4/publication/371169207_Curvature-Aware_Model_Predictive_Contouring_Control/links/64775cecd702370600c50752/Curvature-Aware-Model-Predictive-Contouring-Control.pdf

            # dt = settings["integrator_step"]

            # vel = np.array([v * cd.cos(psi), v * cd.sin(psi)]) # Velocity vector

            # t_vec = np.array([path_dx_normalized, path_dy_normalized])
            # n_vec = np.array([path_dy_normalized, -path_dx_normalized])

            # vt = vel.dot(t_vec) # Velocity path components
            # vn = vel.dot(n_vec)

            # vt_t = vt * dt # Euler integrated velocities
            # vn_t = vn * dt

            # R = 1. / path.get_curvature(s) # max(R) = 1 / 0.0001
            # R = cd.fmax(R, 1e5)

            # # Path velocity
            # s_dot = R * vt * ((R - contour_error - vn_t) + vn_t) / ((R - contour_error - vn_t)**2 + (vt_t)**2)

            # Lorenzo's equations
            path_ddx, path_ddy = path.deriv2(s)
            projection_ratio = 1.0 / (1.0 - ((pos_x - path_x) * path_ddx + (pos_y - path_y) * path_ddy))
            s_dot = v * (cd.cos(psi) * path_dx_normalized + cd.sin(psi) * path_dy_normalized) * projection_ratio
            # Path projection is unnecessary with CA-MPC
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

            if not self.use_ca_mpc:
                cost += terminal_contouring_mp * lag_weight * lag_error**2
                cost += terminal_contouring_mp * contour_weight * contour_error**2
            else:
                cost += terminal_contouring_mp * contour_weight * contour_error_squared
                cost += terminal_contouring_mp * velocity_weight * (s_dot - reference_velocity) ** 2

        return cost


class ContouringModule(ObjectiveModule):

    def __init__(self, settings, num_segments, preview=0.0, use_ca_mpc=True):
        super().__init__()
        self.module_name = "Contouring"  # Needs to correspond to the c++ name of the module
        self.import_name = "contouring.h"
        self.type = "objective"
        if use_ca_mpc:
            self.description = "Curvature-Aware MPC: Tracks a 2D reference path with curvature-aware contouring costs"
        else:
            self.description = "MPCC: Tracks a 2D reference path with contouring costs"

        self.objectives = []
        self.objectives.append(ContouringObjective(settings, num_segments, preview, use_ca_mpc))


# Unused preview cost
# if self.enable_preview and stage_idx == settings["N"] - 1:
#     print(f"Adding preview cost at T = {self.preview} ahead")
#     # In the terminal stage add a preview cost
#     preview_weight = params.get("preview")
#     preview_pos_x, preview_pos_y, preview_s = get_preview_state(model, settings, self.preview)

#     preview_path = Spline2D(params, self.num_segments, preview_s)
#     preview_path_x, preview_path_y = path.at(preview_s)
#     preview_path_dx_normalized, preview_path_dy_normalized = preview_path.deriv_normalized(self.preview)

#     preview_contour_error = preview_path_dy_normalized * (preview_pos_x - preview_path_x) - \
#     preview_path_dx_normalized * (preview_pos_y - preview_path_y)

#     # We use the existing weights here to sort of scale the contribution w.r.t. the regular contouring cost
#     cost += preview_weight * contour_weight * preview_contour_error**2
#     # cost += preview_weight * lag_weight * preview_lag_error**2
