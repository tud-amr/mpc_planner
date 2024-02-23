import os
import sys

import casadi as cd
import numpy as np

from control_modules import ConstraintModule

sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-solver-generator"))


class GaussianConstraintModule(ConstraintModule):

    def __init__(self, settings):
        super().__init__()

        self.n_discs = settings["n_discs"]
        self.max_obstacles = settings["max_obstacles"]

        self.module_name = "GaussianConstraints"
        self.import_name = "gaussian_constraints.h"
        self.description = (
            "Modeling the distribution of obstacle motion as a Gaussian distribution,"
            + " limit the probability of collision using the Gaussian CDF in the linearized collision constraints."
        )

        self.constraints.append(GaussianConstraint(self.n_discs, self.max_obstacles))


class GaussianConstraint:

    def __init__(self, n_discs, max_obstacles):
        self.max_obstacles = max_obstacles
        self.n_discs = n_discs

        self.nh = max_obstacles * n_discs

    def define_parameters(self, params):
        params.add("ego_disc_radius")

        for disc_id in range(self.n_discs):
            params.add(f"ego_disc_{disc_id}_offset")

        for obs_id in range(self.max_obstacles):
            params.add(f"gaussian_obst_{obs_id}_x")
            params.add(f"gaussian_obst_{obs_id}_y")
            # params.add(f"gaussian_obst_{obs_id}_psi")
            params.add(f"gaussian_obst_{obs_id}_major")
            params.add(f"gaussian_obst_{obs_id}_minor")
            params.add(f"gaussian_obst_{obs_id}_risk")
            params.add(f"gaussian_obst_{obs_id}_r")

    def get_lower_bound(self):
        lower_bound = []
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                lower_bound.append(0.0)
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

        # area = model.system.area
        r_vehicle = params.get("ego_disc_radius")

        for obs_id in range(self.max_obstacles):

            obs_x = params.get(f"gaussian_obst_{obs_id}_x")
            obs_y = params.get(f"gaussian_obst_{obs_id}_y")
            obs_pos = np.array([obs_x, obs_y])

            # params.add(f"gaussian_obst_{obs_id}_psi")
            sigma_x = params.get(f"gaussian_obst_{obs_id}_major")
            sigma_y = params.get(f"gaussian_obst_{obs_id}_minor")
            Sigma = np.diag([sigma_x**2, sigma_y**2])

            risk = params.get(f"gaussian_obst_{obs_id}_risk")

            r_obstacle = params.get(f"gaussian_obst_{obs_id}_r")
            combined_radius = r_vehicle + r_obstacle

            diff_pos = pos - obs_pos

            a_ij = diff_pos / cd.sqrt(diff_pos.dot(diff_pos))
            b_ij = combined_radius

            x_erfinv = 1.0 - 2.0 * risk

            # Manual inverse erf, because somehow lacking from casadi...
            # From here: http://casadi.sourceforge.net/v1.9.0/api/internal/d4/d99/casadi_calculus_8hpp_source.html
            z = cd.sqrt(-cd.log((1.0 - x_erfinv) / 2.0))
            y_erfinv = (
                ((1.641345311 * z + 3.429567803) * z - 1.624906493) * z - 1.970840454
            ) / ((1.637067800 * z + 3.543889200) * z + 1.0)

            y_erfinv = y_erfinv - (cd.erf(y_erfinv) - x_erfinv) / (
                2.0 / cd.sqrt(cd.pi) * cd.exp(-y_erfinv * y_erfinv)
            )
            y_erfinv = y_erfinv - (cd.erf(y_erfinv) - x_erfinv) / (
                2.0 / cd.sqrt(cd.pi) * cd.exp(-y_erfinv * y_erfinv)
            )

            constraints.append(
                a_ij.T @ cd.SX(diff_pos)
                - b_ij
                - y_erfinv * cd.sqrt(2.0 * a_ij.T @ Sigma @ a_ij)
            )
        return constraints
