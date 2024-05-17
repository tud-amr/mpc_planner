import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

import copy
import casadi as cd
import numpy as np

from control_modules import ObjectiveModule, Objective

from spline import Spline, Spline2D

def get_preview_state(model, time_ahead):
    # Integrate the trajectory to obtain the preview point
    copied_model = copy.deepcopy(model)

    z_constant_input = cd.vertcat(np.zeros((model.nu)), model.get_x())

    z_preview = model.integrate(z_constant_input, time_ahead)  # Integrate the dynamics T seconds ahead
    z_preview = cd.vertcat(np.zeros((model.nu)), z_preview)

    # Retrieve the resulting position and spline
    copied_model.load(z_preview)
    pos_x = copied_model.get("x")
    pos_y = copied_model.get("y")
    s = copied_model.get("spline")

    return pos_x, pos_y, s


class ContouringObjective:

    def __init__(self, settings, num_segments, preview=0.0):

        self.num_segments = num_segments

        self.enable_preview = preview != 0.0
        self.preview = preview

    def define_parameters(self, params):
        params.add("contour", add_to_rqt_reconfigure=True)
        params.add("lag", add_to_rqt_reconfigure=True)

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
        s = model.get("spline")

        contour_weight = params.get("contour")
        lag_weight = params.get("lag")

        spline = Spline2D(params, self.num_segments, s)
        path_x, path_y = spline.at(s)
        path_dx_normalized, path_dy_normalized = spline.deriv_normalized(s)

        contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
        lag_error = path_dx_normalized * (pos_x - path_x) + path_dy_normalized * (pos_y - path_y)

        # CA-MPC
        # https://www.researchgate.net/profile/Laura-Ferranti-4/publication/371169207_Curvature-Aware_Model_Predictive_Contouring_Control/links/64775cecd702370600c50752/Curvature-Aware-Model-Predictive-Contouring-Control.pdf
        # vel = model.get("v")
        # psi = model.get("psi")

        # v_n = path_dy_normalized * (vel * cd.cos(psi)) - path_dx_normalized * (vel * cd.sin(psi))
        # v_t = -path_dx_normalized * (vel * cd.cos(psi)) - path_dy_normalized * (vel * cd.sin(psi))

        # kappa = splines.get_curvature()
        # R = 1. / kappa

        # theta = cd.atan2((v_t* 0.2) / (R - contour_error - v_n*0.2))

        cost += contour_weight * contour_error**2
        cost += lag_weight * lag_error**2

        # if self.enable_preview and stage_idx == settings["N"] - 1:
        #     print(f"Adding preview cost at T = {self.preview} ahead")
        #     # In the terminal stage add a preview cost
        #     preview_weight = params.get("preview")
        #     preview_pos_x, preview_pos_y, preview_s = get_preview_state(model, self.preview)

        #     preview_splines = MultiSplineXY(params, self.num_segments, preview_s)
        #     preview_path_x, preview_path_y, preview_path_dx_normalized, preview_path_dy_normalized = preview_splines.get_path_and_dpath()

        #     preview_contour_error = preview_path_dy_normalized * (preview_pos_x - preview_path_x) - preview_path_dx_normalized * (
        #         preview_pos_y - preview_path_y
        #     )
        #     preview_lag_error = -preview_path_dx_normalized * (preview_pos_x - preview_path_x) - preview_path_dy_normalized * (
        #         preview_pos_y - preview_path_y
        #     )

        #     # We use the existing weights here to sort of scale the contribution w.r.t. the regular contouring cost
        #     cost += preview_weight * contour_weight * preview_contour_error**2
        #     cost += preview_weight * lag_weight * preview_lag_error**2

        # Boundary penalty
        # boundary_right = WidthSpline(params, self.num_segments, "right", s)
        # br = boundary_right.at(s)
        # cost += 100. * cd.fmax(contour_error - br, 0.)**2

        # boundary_left = WidthSpline(params, self.num_segments, "left", s)
        # bl = boundary_left.at(s)
        # cost += 100. * cd.fmax(bl - contour_error, 0.)**2

        return cost


class ContouringModule(ObjectiveModule):

    def __init__(self, settings, num_segments, preview=0.0):
        super().__init__()
        self.module_name = "Contouring"  # Needs to correspond to the c++ name of the module
        self.import_name = "contouring.h"
        self.type = "objective"
        self.description = "Tracks a 2D reference path with contouring costs"

        self.objectives = []
        self.objectives.append(ContouringObjective(settings, num_segments, preview))