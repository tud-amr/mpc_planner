import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-solver-generator"))

import casadi as cd
import numpy as np

from control_modules import ObjectiveModule, Objective
from contouring import MultiSplineXY


class PathReferenceVelocityObjective:
    """
    Track a reference velocity along the path component that may change with time
    """

    def __init__(self, settings, num_segments):
        self.num_segments = num_segments

    def define_parameters(self, params):
        params.add("velocity")
        params.add("reference_velocity")
        return params

    def get_value(self, model, params, settings, stage_idx):
        psi = model.get("psi")
        v = model.get("v")
        s = model.get("spline")

        velocity_reference = params.get("reference_velocity")
        velocity_weight = params.get("velocity")

        splines = MultiSplineXY(params, self.num_segments, s)
        _, _, path_dx_normalized, path_dy_normalized = splines.get_path_and_dpath()
        path_angle = cd.atan2(path_dy_normalized, path_dx_normalized)
        diff_angle = path_angle - psi

        # Only weigh the component of the velocity in the direction of the path
        v_path = cd.cos(diff_angle) * v

        # track the given velocity reference
        return velocity_weight * ((v_path - velocity_reference) ** 2)


class PathReferenceVelocityModule(ObjectiveModule):

    def __init__(self, settings, num_segments):
        super().__init__()
        self.module_name = "PathReferenceVelocity"
        self.import_name = "path_reference_velocity.h"
        self.type = "objective"
        self.description = "Tracks a velocity in the direction of a 2D path"

        self.objectives = []
        self.objectives.append(PathReferenceVelocityObjective(settings, num_segments))
