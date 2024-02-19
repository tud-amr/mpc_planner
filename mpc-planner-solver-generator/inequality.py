import casadi
import numpy as np
import helpers

""" 
Defines inequality constraints of different types. 
See control_modules.py for their integration into the controller 
"""

"""
DEPRECATED
"""

# Class to aggregate the number of constraints and nh, nparam
class Constraints:

    def __init__(self, params):
        self.upper_bound = []
        self.lower_bound = []
        self.constraints = []

        self.constraint_modules = []

        self.nh = 0
        # self.npar = 0
        self.params = params
        # self.param_idx = param_idx

    def add_constraint(self, constraint):
        self.constraints.append(constraint) # param_idx keeps track of the indices
        constraint.append_upper_bound(self.upper_bound)
        constraint.append_lower_bound(self.lower_bound)

        self.nh += constraint.nh

    def inequality(self, z, param, settings, model):
        result = []

        for constraint in self.constraints:
            constraint.append_constraints(result, z, param, settings, model)

        return result

# Removed "Reuse constraints" because you can now define the parameters by name
# Constraints of the form Ax <= b (+ slack)
class LinearConstraints:

    def __init__(self, params, n_discs, num_constraints, use_slack=False, name="linear_constraint"):
        self.num_constraints = num_constraints
        self.n_discs = n_discs
        self.params = params
        self.name = name
        self.use_slack = use_slack

        self.nh = num_constraints * n_discs

        # @Todo: Be able to add multiple sets of constraints
        for disc in range(n_discs):
            for i in range(num_constraints):
                params.add_parameter(self.constraint_name(disc, i) + "_a1")
                params.add_parameter(self.constraint_name(disc, i) + "_a2")
                params.add_parameter(self.constraint_name(disc, i) + "_b")

    def constraint_name(self, disc_idx, constraint_idx):
        return "disc_"+str(disc_idx)+"_" +self.name +"_"+str(constraint_idx)

    def append_lower_bound(self, lower_bound):
        for scenario in range(0, self.num_constraints):
            for disc in range(0, self.n_discs):
                lower_bound.append(-np.inf)

    def append_upper_bound(self, upper_bound):
        for scenario in range(0, self.num_constraints):
            for disc in range(0, self.n_discs):
                upper_bound.append(0.0)

    def append_constraints(self, constraints, z, param, settings, model):
        settings.params.load_params(param)

        # States
        pos_x = model.get_state(z, 'x', True)
        pos_y = model.get_state(z, 'y', True)
        pos = np.array([pos_x, pos_y])

        psi = model.get_state(z, 'psi', True)

        slack = 0.
        if self.use_slack:
            slack = model.get_state(z, 'slack', True)

        rotation_car = helpers.rotation_matrix(psi)
        area = model.system.area
        for disc_it in range(0, self.n_discs):
            disc_x = area.offsets[disc_it]
            disc_relative_pos = np.array([disc_x, 0])
            disc_pos = pos + rotation_car.dot(disc_relative_pos)

            # A'x <= b
            for constraint_it in range(0, self.num_constraints):
                a1 = getattr(settings.params, self.constraint_name(disc_it, constraint_it) + "_a1")
                a2 = getattr(settings.params, self.constraint_name(disc_it, constraint_it) + "_a2")
                b =  getattr(settings.params, self.constraint_name(disc_it, constraint_it) + "_b")

                if not self.use_slack:
                    constraints.append(a1 * disc_pos[0] + a2 * disc_pos[1] - b)
                else:
                    constraints.append(a1 * disc_pos[0] + a2 * disc_pos[1] - (b + slack)) # d = slack


# Constraints of the form Ax <= b (+ slack)
class LinearConstraintsY:

    def __init__(self, n_discs, lb, ub):
        self.n_discs = n_discs

        self.lb = lb
        self.ub = ub

        self.nh = n_discs
        print('Warning: Adding Y constraints, make sure the values are appropriate for the scenario!')

    def append_lower_bound(self, lower_bound):
        for disc in range(0, self.n_discs):
            lower_bound.append(self.lb)

    def append_upper_bound(self, upper_bound):
        for disc in range(0, self.n_discs):
            upper_bound.append(self.ub)

    def append_constraints(self, constraints, z, param, settings, model):
        # States
        pos_x = model.get_state(z, 'x', True)
        pos_y = model.get_state(z, 'y', True)
        pos = np.array([pos_x, pos_y])

        psi = model.get_state(z, 'psi', True)

        rotation_car = helpers.rotation_matrix(psi)
        area = model.system.area
        for disc_it in range(0, self.n_discs):
            disc_x = area.offsets[disc_it]
            disc_relative_pos = np.array([disc_x, 0])
            disc_pos = pos + rotation_car.dot(disc_relative_pos)

            # Simple but not ideal
            constraints.append(-disc_pos[1])  # lb <= disc_y <= ub (y in carla is negative...)
            # constraints.append(disc_pos[1] - 3)
            # constraints.append(-disc_pos[1] + 3)
            # constraints.append(disc_pos[1] - 325.704) # y <= 335.707
            # constraints.append(-disc_pos[1] + 331.347) # -y <= -331.347, y >= 331.347


class RoomConstraints:

    def __init__(self, n_discs, x_lb, x_ub, y_lb, y_ub, x_offset, y_offset):
        self.n_discs = n_discs

        self.x_lb = x_lb
        self.x_ub = x_ub
        self.y_lb = y_lb
        self.y_ub = y_ub
        self.x_offset = x_offset
        self.y_offset = y_offset

        self.nh = 2*n_discs

    def append_lower_bound(self, lower_bound):
        for disc in range(0, self.n_discs):
            lower_bound.append(self.x_lb)
        for disc in range(0, self.n_discs):
            lower_bound.append(self.y_lb)

    def append_upper_bound(self, upper_bound):
        for disc in range(0, self.n_discs):
            upper_bound.append(self.x_ub)

        for disc in range(0, self.n_discs):
            upper_bound.append(self.y_ub)

    def append_constraints(self, constraints, z, param, settings, model):
        # States
        pos_x = model.get_state(z, 'x', True)
        pos_y = model.get_state(z, 'y', True)
        pos = np.array([pos_x, pos_y])

        psi = model.get_state(z, 'psi', True)

        rotation_car = helpers.rotation_matrix(psi)
        area = model.system.area
        for disc_it in range(0, self.n_discs):
            disc_x = area.offsets[disc_it]
            disc_relative_pos = np.array([disc_x, 0])
            disc_pos = pos + rotation_car.dot(disc_relative_pos)

            # Simple but not ideal
            constraints.append(disc_pos[0] - self.x_offset)  # lb <= disc_x <= ub
            constraints.append(disc_pos[1] - self.y_offset)  # lb <= disc_y <= ub


# Constraints of the form A'x'A <= 0 (+ slack)
class CircularConstraints:

    def __init__(self, n_discs, max_obstacles, params, rotation_clockwise=True):
        self.max_obstacles = max_obstacles
        self.n_discs = n_discs

        self.nh = max_obstacles * n_discs

        # Add parameters
        for obs_id in range(max_obstacles):
            params.add_parameter("circular_obst_" + str(obs_id) + "_x")
            params.add_parameter("circular_obst_" + str(obs_id) + "_y")

        self.rotation_clockwise = rotation_clockwise

    def append_lower_bound(self, lower_bound):
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                lower_bound.append(1.0)

    def append_upper_bound(self, upper_bound):
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                upper_bound.append(np.Inf)

    def append_constraints(self, constraints, z, param, settings, model):

        settings.params.load_params(param)

        # States
        pos_x = model.get_state(z, 'x', True)
        pos_y = model.get_state(z, 'y', True)
        pos = np.array([pos_x, pos_y])

        psi = model.get_state(z, 'psi', True)
        # slack = model.get_input(z, 'slack', True)

        rotation_car = helpers.rotation_matrix(psi)

        # r_disc = getattr(settings.params, 'disc_r') #param[self.start_param]
        area = model.system.area
        r_disc = area.radius

        # Constraint for dynamic obstacles
        for obstacle_it in range(0, self.max_obstacles):
            # Retrieve parameters
            obst_x = getattr(settings.params, "circular_obst_" + str(obstacle_it) + "_x")
            obst_y = getattr(settings.params, "circular_obst_" + str(obstacle_it) + "_y")

            # obstacle computations
            obstacle_cog = np.array([obst_x, obst_y])

            ab = np.array([[1. / (r_disc**2.), 0.],
                            [0., 1. / (r_disc**2.) ]])

            for disc_it in range(0, self.n_discs):
                # Get and compute the disc position
                disc_x = area.offsets[disc_it]
                disc_relative_pos = np.array([disc_x, 0])
                disc_pos = pos + rotation_car.dot(disc_relative_pos)

                # construct the constraint and append it
                disc_to_obstacle = disc_pos - obstacle_cog
                c_disc_obstacle = disc_to_obstacle.transpose().dot(ab).dot(disc_to_obstacle)
                constraints.append(c_disc_obstacle)# + slack)

# Constraints of the form A'x'A <= 0 (+ slack)
class EllipsoidConstraints:

    def __init__(self, n_discs, max_obstacles, params, rotation_clockwise=True):
        self.max_obstacles = max_obstacles
        self.n_discs = n_discs

        self.nh = max_obstacles * n_discs

        # Add parameters
        for obs_id in range(max_obstacles):
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_x")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_y")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_psi")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_major")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_minor")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_chi")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_r")

        self.rotation_clockwise = rotation_clockwise

    def append_lower_bound(self, lower_bound):
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                # lower_bound.append(-np.inf)
                lower_bound.append(1.0)

    def append_upper_bound(self, upper_bound):
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                # upper_bound.append(0.)
                upper_bound.append(np.Inf)

    def append_constraints(self, constraints, z, param, settings, model):

        settings.params.load_params(param)

        # States
        pos_x = model.get_state(z, 'x', True)
        pos_y = model.get_state(z, 'y', True)
        pos = np.array([pos_x, pos_y])

        psi = model.get_state(z, 'psi', True)
        # slack = model.get_input(z, 'slack', True)

        rotation_car = helpers.rotation_matrix(psi)

        area = model.system.area
        r_disc = area.radius

        # Constraint for dynamic obstacles
        for obstacle_it in range(0, self.max_obstacles):
            # Retrieve parameters
            obst_x = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_x")
            obst_y = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_y")
            obst_psi = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_psi")
            obst_major = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_major")
            obst_minor = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_minor")
            obst_r = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_r")

            # multiplier for the risk when obst_major, obst_major only denote the covariance
            # (i.e., the smaller the risk, the larger the ellipsoid).
            # This value should already be a multiplier (using exponential cdf).
            chi = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_chi")

            # obstacle computations
            obstacle_cog = np.array([obst_x, obst_y])

            # Compute ellipse matrix
            obst_major *= casadi.sqrt(chi)
            obst_minor *= casadi.sqrt(chi)
            ab = np.array([[1. / ((obst_major + (r_disc + obst_r)) ** 2), 0],
                           [0, 1. / ((obst_minor + (r_disc + obst_r)) ** 2)]])

            # In the original LMPCC paper the angle of the obstacles is defined clockwise
            # While it could also make sense to define this anti-clockwise, just like the orientation of the Roboat
            if self.rotation_clockwise:
                obstacle_rotation = helpers.rotation_matrix(obst_psi)
            else:
                obstacle_rotation = helpers.rotation_matrix(-obst_psi)

            obstacle_ellipse_matrix = obstacle_rotation.transpose().dot(ab).dot(obstacle_rotation)

            for disc_it in range(0, self.n_discs):
                # Get and compute the disc position
                disc_x = area.offsets[disc_it]
                disc_relative_pos = np.array([disc_x, 0])
                disc_pos = pos + rotation_car.dot(disc_relative_pos)

                # construct the constraint and append it
                disc_to_obstacle = disc_pos - obstacle_cog
                c_disc_obstacle = disc_to_obstacle.transpose().dot(obstacle_ellipse_matrix).dot(disc_to_obstacle)
                constraints.append(c_disc_obstacle)# + slack)


# Constraints based on the ellipsoidal constraints, but assuming that both the robot and obstacles are discs
# Then linearizing (A^Tx <= b) orthogonal to the vector from the obstacle to the robot. A and b are computed here.
# All data is equal to the ellipsoidal constraints (only a linearization is added at the end)
class EllipsoidLinearizedConstraints:

    def __init__(self, n_discs, max_obstacles, params, rotation_clockwise=True):
        self.max_obstacles = max_obstacles
        self.n_discs = n_discs

        self.nh = max_obstacles * n_discs

        # Add parameters
        for obs_id in range(max_obstacles):
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_x")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_y")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_psi")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_major")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_minor")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_chi")
            params.add_parameter("ellipsoid_obst_" + str(obs_id) + "_r")

        self.rotation_clockwise = rotation_clockwise

    def append_lower_bound(self, lower_bound):
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                lower_bound.append(-np.inf)
                # lower_bound.append(1.0)

    def append_upper_bound(self, upper_bound):
        for obs in range(0, self.max_obstacles):
            for disc in range(0, self.n_discs):
                upper_bound.append(0.)
                # upper_bound.append(np.Inf)


    def append_constraints(self, constraints, z, param, settings, model):

        settings.params.load_params(param)

        # Retrieve variables
        x = z[model.nu:model.nu + model.nx]
        u = z[0:model.nu]

        # States
        pos_x = model.get_state(z, 'x', True)
        pos_y = model.get_state(z, 'y', True)
        pos = np.array([pos_x, pos_y])

        psi = model.get_state(z, 'psi', True)

        rotation_car = helpers.rotation_matrix(psi)

        area = model.system.area
        r_disc = area.radius

        # Constraint for dynamic obstacles
        for obstacle_it in range(0, self.max_obstacles):
            # Retrieve parameters
            obst_x = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_x")
            obst_y = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_y")
            obst_psi = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_psi")
            obst_major = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_major")
            obst_minor = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_minor")
            obst_r = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_r")

            # multiplier for the risk when obst_major, obst_major only denote the covariance
            # (i.e., the smaller the risk, the larger the ellipsoid).
            # This value should already be a multiplier (using exponential cdf).
            chi = getattr(settings.params, "ellipsoid_obst_" + str(obstacle_it) + "_chi")

            # obstacle computations
            obstacle_cog = np.array([obst_x, obst_y])

            # Compute ellipse matrix
            obst_major *= casadi.sqrt(chi)
            obst_minor *= casadi.sqrt(chi)
            ab = np.array([[1. / ((obst_major + (r_disc + obst_r)) ** 2), 0],
                           [0, 1. / ((obst_minor + (r_disc + obst_r)) ** 2)]])

            # In the original LMPCC paper the angle of the obstacles is defined clockwise
            # While it could also make sense to define this anti-clockwise, just like the orientation of the Roboat
            if self.rotation_clockwise:
                obstacle_rotation = helpers.rotation_matrix(obst_psi)
            else:
                obstacle_rotation = helpers.rotation_matrix(-obst_psi)

            obstacle_ellipse_matrix = obstacle_rotation.transpose().dot(ab).dot(obstacle_rotation)

            for disc_it in range(0, self.n_discs):
                # Get and compute the disc position
                disc_x = area.offsets[disc_it]
                disc_relative_pos = np.array([disc_x, 0])
                disc_pos = pos + rotation_car.dot(disc_relative_pos)

                # construct the constraint and append it
                disc_to_obstacle = disc_pos - obstacle_cog
                c_disc_obstacle = disc_to_obstacle.transpose().dot(obstacle_ellipse_matrix).dot(disc_to_obstacle)

                obs_pos = np.array([obst_x, obst_y])
                A = (obs_pos - disc_pos) / casadi.norm_2(disc_pos - obs_pos)
                b = A.T @ (obs_pos.T - A*(obst_r + r_disc))
                constraints.append(A.T@disc_pos - b)


# Constraints based on Hai's work
# Todo: Add support for multiple discs
class GaussianConstraints:
    def __init__(self, n_discs, max_obstacles, params):
        self.n_discs = n_discs
        self.max_obstacles = max_obstacles

        if n_discs > 1:
            raise NotImplementedError("GaussianConstraints for more than 1 disc is not implemented!")

        self.nh = self.n_discs * self.max_obstacles
        params.add_parameter("sigma_x")
        params.add_parameter("sigma_y")
        params.add_parameter("epsilon")
        params.add_parameter("r_obstacle")

        for obst_id in range(max_obstacles):
            params.add_parameter("obst_" + str(obst_id) + "_x")
            params.add_parameter("obst_" + str(obst_id) + "_y")

        # print(self)

    def __str__(self):
        return "[" + str(self.start_param) + "]: Gaussian Constraints for " + str(self.max_obstacles) + " obstacles, " + str(self.n_discs) + " vehicle discs.\n"

    def append_lower_bound(self, lower_bound):
        for obs in range(0, self.max_obstacles):
            lower_bound.append(0.)

    def append_upper_bound(self, upper_bound):
        for obs in range(0, self.max_obstacles):
            upper_bound.append(np.Inf)

    def append_constraints(self, constraints, z, param, settings, model):
        settings.params.load_params(param)

        # States
        pos_x = model.get_state(z, 'x', True)
        pos_y = model.get_state(z, 'y', True)
        pos = np.array([pos_x, pos_y])

        # Retrieve covariance
        sigma_x = getattr(settings.params, "sigma_x")
        sigma_y = getattr(settings.params, "sigma_y")
        Sigma = np.diag([sigma_x**2, sigma_y**2])

        epsilon = getattr(settings.params, "epsilon")

        area = model.system.area
        r_vehicle = area.radius
        r_obstacle = getattr(settings.params, "r_obstacle")
        combined_radius = r_vehicle + r_obstacle

        for v in range(self.max_obstacles):
            obs_x = getattr(settings.params, "obst_" + str(v) + "_x")
            obs_y = getattr(settings.params, "obst_" + str(v) + "_y")

            obs_pos = np.array([obs_x, obs_y])
            diff_pos = pos - obs_pos

            a_ij = diff_pos / casadi.sqrt(diff_pos.dot(diff_pos))
            b_ij = combined_radius

            x_erfinv = 1. - 2. * epsilon

            # Manual inverse erf, because somehow lacking from casadi...
            # From here: http: // casadi.sourceforge.net / v1.9.0 / api / internal / d4 / d99 / casadi__calculus_8hpp_source.html  # l00307
            z = casadi.sqrt(-casadi.log((1.0-x_erfinv) / 2.0))
            y_erfinv = (((1.641345311 * z + 3.429567803) * z - 1.624906493) * z - 1.970840454) / \
                       ((1.637067800 * z + 3.543889200) * z + 1.0)

            y_erfinv = y_erfinv - (casadi.erf(y_erfinv) - x_erfinv) / (2.0 / casadi.sqrt(casadi.pi) * casadi.exp(-y_erfinv * y_erfinv))
            y_erfinv = y_erfinv - (casadi.erf(y_erfinv) - x_erfinv) / (2.0 / casadi.sqrt(casadi.pi) * casadi.exp(-y_erfinv * y_erfinv))

            constraints.append(a_ij.T@casadi.SX(diff_pos) - b_ij - y_erfinv * casadi.sqrt(2.*a_ij.T @ Sigma @ a_ij))