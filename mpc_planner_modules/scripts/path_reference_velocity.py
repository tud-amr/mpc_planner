import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

import casadi as cd
import numpy as np

from control_modules import ObjectiveModule, Objective
from contouring import Spline, Spline2D

class PathReferenceVelocityObjective:
    """
    Track a reference velocity along the path component that may change with time
    """

    def __init__(self, settings):
        self.num_segments = settings["contouring"]["num_segments"]

    def define_parameters(self, params):

        for i in range(self.num_segments):
            params.add(f"spline_v{i}_a", bundle_name="spline_v_a")
            params.add(f"spline_v{i}_b", bundle_name="spline_v_b")
            params.add(f"spline_v{i}_c", bundle_name="spline_v_c")
            params.add(f"spline_v{i}_d", bundle_name="spline_v_d")

        return params

    def get_value(self, model, params, settings, stage_idx):
        # The cost is computed in the contouring cost
        return 0.0


class PathReferenceVelocityModule(ObjectiveModule):

    def __init__(self, settings):
        super().__init__()
        self.module_name = "PathReferenceVelocity"
        self.import_name = "path_reference_velocity.h"
        self.type = "objective"
        self.description = "Tracks a dynamic velocity reference along the path"

        self.objectives = []
        self.objectives.append(PathReferenceVelocityObjective(settings))
