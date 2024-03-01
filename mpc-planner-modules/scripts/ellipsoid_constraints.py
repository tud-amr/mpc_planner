import os
import sys

import casadi as cd
import numpy as np

from util.math import rotation_matrix
from control_modules import ConstraintModule

sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-solver-generator"))


class EllipsoidConstraintModule(ConstraintModule):

    def __init__(self, settings):
        super().__init__()

        self.n_discs = settings["n_discs"]
        self.max_obstacles = settings["max_obstacles"]

        self.module_name = "EllipsoidConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "ellipsoid_constraints.h"
        self.description = "Avoid obstacles, modeled as ellipsoids (possibly including Gaussian noise)."

        self.constraints.append(EllipsoidConstraint(self.n_discs, self.max_obstacles))


class EllipsoidConstraint:

    def __init__(self, n_discs, max_obstacles):
        self.max_obstacles = max_obstacles
        self.n_discs = n_discs

        self.nh = max_obstacles * n_discs

    def define_parameters(self, params):
        params.add("ego_disc_radius")

        for disc_id in range(self.n_discs):
            params.add(f"ego_disc_{disc_id}_offset")

        for obs_id in range(self.max_obstacles):
            params.add(f"ellipsoid_obst_{obs_id}_x")
            params.add(f"ellipsoid_obst_{obs_id}_y")
            params.add(f"ellipsoid_obst_{obs_id}_psi")
            params.add(f"ellipsoid_obst_{obs_id}_major")
            params.add(f"ellipsoid_obst_{obs_id}_minor")
            params.add(f"ellipsoid_obst_{obs_id}_chi")
            params.add(f"ellipsoid_obst_{obs_id}_r")

    def get_lower_bound(self):
        lower_bound = []
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                lower_bound.append(1.0)
        return lower_bound

    def get_upper_bound(self):
        upper_bound = []
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                upper_bound.append(np.Inf)
        return upper_bound

    def get_constraints(self, model, params, settings, stage_idx):
        constraints = []
        pos_x = model.get("x")
        pos_y = model.get("y")
        pos = np.array([pos_x, pos_y])

        psi = model.get("psi")
        # slack = model.get('slack')

        rotation_car = rotation_matrix(psi)

        r_disc = params.get("ego_disc_radius")

        # Constraint for dynamic obstacles
        for obs_id in range(0, self.max_obstacles):
            obst_x = params.get(f"ellipsoid_obst_{obs_id}_x")
            obst_y = params.get(f"ellipsoid_obst_{obs_id}_y")
            obstacle_cog = np.array([obst_x, obst_y])

            obst_psi = params.get(f"ellipsoid_obst_{obs_id}_psi")
            obst_major = params.get(f"ellipsoid_obst_{obs_id}_major")
            obst_minor = params.get(f"ellipsoid_obst_{obs_id}_minor")
            obst_r = params.get(f"ellipsoid_obst_{obs_id}_r")

            # multiplier for the risk when obst_major, obst_major only denote the covariance
            # (i.e., the smaller the risk, the larger the ellipsoid).
            # This value should already be a multiplier (using exponential cdf).
            chi = params.get(f"ellipsoid_obst_{obs_id}_chi")

            # Compute ellipse matrix
            obst_major *= cd.sqrt(chi)
            obst_minor *= cd.sqrt(chi)
            ab = np.array(
                [
                    [1.0 / ((obst_major + (r_disc + obst_r)) ** 2), 0],
                    [0, 1.0 / ((obst_minor + (r_disc + obst_r)) ** 2)],
                ]
            )

            obstacle_rotation = rotation_matrix(obst_psi)
            obstacle_ellipse_matrix = obstacle_rotation.transpose().dot(ab).dot(obstacle_rotation)

            for disc_it in range(0, self.n_discs):
                # Get and compute the disc position
                # disc_x = area.offsets[disc_it]
                disc_x = params.get(f"ego_disc_{disc_it}_offset")
                disc_relative_pos = np.array([disc_x, 0])
                disc_pos = pos + rotation_car.dot(disc_relative_pos)

                # construct the constraint and append it
                disc_to_obstacle = disc_pos - obstacle_cog
                c_disc_obstacle = disc_to_obstacle.transpose().dot(obstacle_ellipse_matrix).dot(disc_to_obstacle)
                constraints.append(c_disc_obstacle)  # + slack)

        return constraints
