"""
Homotopic path search in the state space for generating guidance trajectories
In the "Constraint" version, we use these trajectories to initialize the planner and
to linearize the collision avoidance constraints
"""

import numpy as np

import sys
import os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))


from util.math import rotation_matrix
from util.code_generation import add_definition
from control_modules import ConstraintModule

from ellipsoid_constraints import EllipsoidConstraint
from ellipsoid_constraints import EllipsoidConstraintModule


class GuidanceConstraintModule(ConstraintModule):

    def __init__(self, settings, constraint_submodule=None):
        super().__init__()
        if constraint_submodule is None:
            self.constraint_submodule = EllipsoidConstraintModule
        else:
            self.constraint_submodule = constraint_submodule

        self.module_name = "GuidanceConstraints"  # c++ name of the module
        self.import_name = "guidance_constraints.h"

        self.dependencies.append("guidance_planner")

        self.constraints.append(
            LinearConstraints(
                max_obstacles=settings["max_obstacles"], other_halfspaces=settings["linearized_constraints"]["add_halfspaces"]
            )
        )
        self.sources.append("linearized_constraints.h")

        # Initialize the underlying constraints
        self.constraint_submodule = self.constraint_submodule(settings)
        self.constraints.append(self.constraint_submodule.constraints[-1])
        self.sources.append(self.constraint_submodule.import_name)

        # LinearConstraints(max_obstacles + static_obstacles))
        # self.add_submodule(constraint_submodule(params, n_discs, max_obstacles))

        self.description = "Optimize several trajectories in parallel using "
        "constraints linearized w.r.t. guidance trajectories\n"
        ("\t\tConstraint for the underlying solver: " + self.constraint_submodule.description)

    def add_definitions(self, header_file):
        header_file.write("#include <mpc_planner_modules/" + self.constraint_submodule.import_name + ">\n")
        add_definition(
            header_file,
            "GUIDANCE_CONSTRAINTS_TYPE",
            self.constraint_submodule.module_name,
        )


# Constraints of the form Ax <= b (+ slack)
# NOTE: For guidance we only need a single linear constraint for the robot!
class LinearConstraints:

    def __init__(self, max_obstacles, other_halfspaces=0):
        self.max_obstacles = max_obstacles
        self.nh = self.max_obstacles + other_halfspaces

    def define_parameters(self, params):

        for index in range(self.nh):
            params.add(self.constraint_name(index) + "_a1", bundle_name="lin_constraint_a1")
            params.add(self.constraint_name(index) + "_a2", bundle_name="lin_constraint_a2")
            params.add(self.constraint_name(index) + "_b", bundle_name="lin_constraint_b")

    def constraint_name(self, index):
        return f"lin_constraint_{index}"

    def get_lower_bound(self):
        lower_bound = []
        for index in range(0, self.nh):
            lower_bound.append(-np.inf)
        return lower_bound

    def get_upper_bound(self):
        upper_bound = []
        for index in range(0, self.nh):
            upper_bound.append(0.0)
        return upper_bound

    def get_constraints(self, model, params, settings, stage_idx):
        constraints = []

        # States
        pos_x = model.get("x")
        pos_y = model.get("y")
        pos = np.array([pos_x, pos_y])

        for index in range(self.nh):
            a1 = params.get(self.constraint_name(index) + "_a1")
            a2 = params.get(self.constraint_name(index) + "_a2")
            b = params.get(self.constraint_name(index) + "_b")

            constraints.append(a1 * pos[0] + a2 * pos[1] - b)

        return constraints
