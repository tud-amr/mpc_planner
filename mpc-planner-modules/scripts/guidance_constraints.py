"""
Homotopic path search in the state space for generating guidance trajectories
In the "Constraint" version, we use these trajectories to initialize the planner and
to linearize the collision avoidance constraints
"""

import numpy as np

import sys
import os

sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-solver-generator"))


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
                n_discs=settings["n_discs"], max_obstacles=settings["max_obstacles"]
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
        (
            "\t\tConstraint for the underlying solver: "
            + self.constraint_submodule.description
        )

    def add_definitions(self, header_file):
        header_file.write(
            "#include <mpc-planner-modules/"
            + self.constraint_submodule.import_name
            + ">\n"
        )
        add_definition(
            header_file,
            "GUIDANCE_CONSTRAINTS_TYPE",
            self.constraint_submodule.module_name,
        )


# Constraints of the form Ax <= b (+ slack)
class LinearConstraints:

    def __init__(self, n_discs, max_obstacles):
        self.n_constraints = max_obstacles
        self.n_discs = n_discs

        assert self.n_discs == 1, "Only one disc is supported for now"

        self.nh = self.n_constraints

    def define_parameters(self, params):

        for disc_id in range(self.n_discs):
            params.add(f"ego_disc_{disc_id}_offset")

        for index in range(self.n_constraints):
            params.add(self.constraint_name(index) + "_a1")
            params.add(self.constraint_name(index) + "_a2")
            params.add(self.constraint_name(index) + "_b")

    def constraint_name(self, index):
        return f"lin_constraint_{index}"

    def get_lower_bound(self):
        lower_bound = []
        for index in range(0, self.n_constraints):
            lower_bound.append(-np.inf)
        return lower_bound

    def get_upper_bound(self):
        upper_bound = []
        for index in range(0, self.n_constraints):
            upper_bound.append(0.0)
        return upper_bound

    def get_constraints(self, model, params, settings, stage_idx):
        constraints = []

        # States
        pos_x = model.get("x")
        pos_y = model.get("y")
        pos = np.array([pos_x, pos_y])
        psi = model.get("psi")

        rotation_car = rotation_matrix(psi)
        for disc_it in range(0, self.n_discs):
            disc_x = params.get(f"ego_disc_{disc_it}_offset")
            disc_relative_pos = np.array([disc_x, 0])
            disc_pos = pos + rotation_car.dot(disc_relative_pos)

            for index in range(self.n_constraints):
                a1 = params.get(self.constraint_name((index)) + "_a1")
                a2 = params.get(self.constraint_name((index)) + "_a2")
                b = params.get(self.constraint_name((index)) + "_b")

                constraints.append(a1 * disc_pos[0] + a2 * disc_pos[1] - b)

        return constraints
