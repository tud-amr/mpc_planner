import casadi as cd
import numpy as np

from util.files import model_map_path, write_to_yaml

from spline import Spline2D

# Returns discretized dynamics of a given model (see below)
def forces_discrete_dynamics(z, p, model, settings, nx = None):
    import forcespro.nlp

    """
    @param z: state vector (u, x)
    @param p: parameters
    @param model: Model of the system
    @param settings: Integrator stepsize in seconds
    @param nx: Number of states (can be used to modify which states are integrated)
    @return:
    """
    # We use an explicit RK4 integrator here to discretize continuous dynamics

    if nx is None:
        nx = model.nx

    return forcespro.nlp.integrate(
        model.continuous_model,
        z[model.nu : model.nu + nx],
        z[0 : model.nu],
        integrator=forcespro.nlp.integrators.RK4,
        stepsize=settings["integrator_step"],
    )


def numpy_to_casadi(x: np.array) -> cd.SX:
    result = None
    for param in x:
        if result is None:
            result = param
        else:
            result = cd.vertcat(result, param)
    return result


class DynamicsModel:

    def __init__(self):
        self.nu = 0  # number of control variables
        self.nx = 0  # number of states

        self.states = []
        self.inputs = []

        self.lower_bound = []
        self.upper_bound = []

        self.params = None
        self.nx_integrate = None

    def discrete_dynamics(self, z, p, settings):
        params = settings["params"]
        params.load(p)
        self.load(z)
        self.load_settings(settings)

        nx_integrate = self.nx if self.nx_integrate is None else self.nx_integrate
        integrated_states = forces_discrete_dynamics(z, p, self, settings, nx=nx_integrate)

        integrated_states = self.model_discrete_dynamics(z, integrated_states)
        return integrated_states

    def model_discrete_dynamics(self, z, integrated_states):
        return integrated_states

    def get_nvar(self):
        return self.nu + self.nx

    def get_xinit(self):
        return range(self.nu, self.get_nvar())

    def acados_symbolics(self):
        x = cd.SX.sym("x", self.nx)  # [px, py, vx, vy]
        u = cd.SX.sym("u", self.nu)  # [ax, ay]
        z = cd.vertcat(u, x)
        self.load(z)
        return z

    def get_acados_dynamics(self):
        self._x_dot = cd.SX.sym("x_dot", self.nx)

        f_expl = numpy_to_casadi(self.continuous_model(self._z[self.nu :], self._z[: self.nu]))
        f_impl = self._x_dot - f_expl
        return f_expl, f_impl

    def get_x(self):
        return self._z[self.nu :]
    
    def get_u(self):
        return self._z[:self.nu]

    def get_acados_x_dot(self):
        return self._x_dot

    def get_acados_u(self):
        return self._z[: self.nu]

    def load(self, z):
        self._z = z

    def load_settings(self, settings):
        self.params = settings["params"]
        self.settings = settings

    def save_map(self):
        file_path = model_map_path()

        map = dict()
        for idx, state in enumerate(self.states):
            map[state] = ["x", idx + self.nu, self.get_bounds(state)[0], self.get_bounds(state)[1]]

        for idx, input in enumerate(self.inputs):
            map[input] = ["u", idx, self.get_bounds(input)[0], self.get_bounds(input)[1]]

        write_to_yaml(file_path, map)

    # def integrate(self, z, duration):
        # return discrete_dynamics(z, self, duration)

    def do_not_use_integration_for_last_n_states(self, n):
        self.nx_integrate = self.nx - n

    def get(self, state_or_input):
        if state_or_input in self.states:
            i = self.states.index(state_or_input)
            return self._z[self.nu + i]
        elif state_or_input in self.inputs:
            i = self.inputs.index(state_or_input)
            return self._z[i]
        else:
            raise IOError(f"Requested a state or input `{state_or_input}' that was neither a state nor an input for the selected model")

    def set_bounds(self, lower_bound, upper_bound):
        assert len(lower_bound) == len(upper_bound) == len(self.lower_bound)
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def get_bounds(self, state_or_input):
        if state_or_input in self.states:
            i = self.states.index(state_or_input)
            return (
                self.lower_bound[self.nu + i],
                self.upper_bound[self.nu + i],
                self.upper_bound[self.nu + i] - self.lower_bound[self.nu + i],
            )
        elif state_or_input in self.inputs:
            i = self.inputs.index(state_or_input)
            return (
                self.lower_bound[i],
                self.upper_bound[i],
                self.upper_bound[i] - self.lower_bound[i],
            )
        else:
            raise IOError(f"Requested a state or input `{state_or_input}' that was neither a state nor an input for the selected model")


class SecondOrderUnicycleModel(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2  # number of control variables
        self.nx = 4  # number of states

        self.states = ["x", "y", "psi", "v"]
        self.inputs = ["a", "w"]

        self.lower_bound = [-2.0, -2.0, -200.0, -200.0, -np.pi * 4, -2.0]
        self.upper_bound = [2.0, 2.0, 200.0, 200.0, np.pi * 4, 3.0]

    def continuous_model(self, x, u):

        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]

        return np.array([v * cd.cos(psi), v * cd.sin(psi), w, a])


class ContouringSecondOrderUnicycleModel(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2  # number of control variables
        self.nx = 5  # number of states

        self.states = ["x", "y", "psi", "v", "spline"]
        self.inputs = ["a", "w"]

        self.lower_bound = [-4.0, -0.8, -2000.0, -2000.0, -np.pi * 4, -0.01, -1.0]
        self.upper_bound = [4.0, 0.8, 2000.0, 2000.0, np.pi * 4, 3.0, 10000.0]

    def continuous_model(self, x, u):

        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]

        return np.array([v * cd.cos(psi), v * cd.sin(psi), w, a, v])


class ContouringSecondOrderUnicycleModelWithSlack(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2  # number of control variables
        self.nx = 6  # number of states

        self.states = ["x", "y", "psi", "v", "spline", "slack"]
        self.inputs = ["a", "w"]

        self.lower_bound = [-2.0, -0.8, -2000.0, -2000.0, -np.pi * 4, -0.01, -1.0, 0.0]
        self.upper_bound = [2.0, 0.8, 2000.0, 2000.0, np.pi * 4, 3.0, 10000.0, 5000.0]

    def continuous_model(self, x, u):

        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]

        return np.array([v * cd.cos(psi), v * cd.sin(psi), w, a, v, 0.0])

    # NOTE: No initialization for slack variable
    def get_xinit(self):
        return range(self.nu, self.get_nvar() - 1)


# Bicycle model with dynamic steering
class BicycleModel2ndOrder(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2
        self.nx = 6

        self.states = ["x", "y", "psi", "v", "delta", "spline"]
        self.inputs = ["a", "w"]

        # Prius limits: https://github.com/oscardegroot/lmpcc/blob/prius/lmpcc_solver/scripts/systems.py
        # w [-0.2, 0.2] | a [-1.0 1.0]
        # w was 0.5
        # delta was 0.45

        # NOTE: the angle of the vehicle should not be limited to -pi, pi, as the solution will not shift when it is at the border!
        # a was 3.0
        self.lower_bound = [-6.0, -1.5, -1.0e6, -1.0e6, -np.pi * 4, -0.01, -0.45, -1.0]
        self.upper_bound = [6.0, 1.5, 1.0e6, 1.0e6, np.pi * 4, 8.0, 0.45, 5000.0]

    def continuous_model(self, x, u):
        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        delta = x[4]

        wheel_base = 2.79  # between front wheel center and rear wheel center
        wheel_tread = 1.64  # between left wheel center and right wheel center
        front_overhang = 1.0  # between front wheel center and vehicle front
        rear_overhang = 1.1  # between rear wheel center and vehicle rear
        left_overhang = 0.128  # between left wheel center and vehicle left
        right_overhang = 0.128  # between right wheel center and vehicle right

        # self.length = front_overhang + rear_overhang + wheel_base #4.54
        # self.width = wheel_tread + left_overhang + right_overhang #2.25

        # NOTE: Is at the rear wheel center.
        # This defines where it is w.r.t. the back
        # self.com_to_back = self.length/2.

        # NOTE: Mass is equally distributed according to the parameters
        lr = wheel_base / 2.0
        lf = wheel_base / 2.0
        ratio = lr / (lr + lf)

        beta = cd.arctan(ratio * cd.tan(delta))

        return np.array([v * cd.cos(psi + beta), v * cd.sin(psi + beta), (v / lr) * cd.sin(beta), a, w, v])

# Bicycle model with dynamic steering
class BicycleModel2ndOrderCurvatureAware(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2
        self.nx = 6

        self.states = ["x", "y", "psi", "v", "delta", "spline"]
        self.inputs = ["a", "w"]

        self.do_not_use_integration_for_last_n_states(n=1)

        # Prius limits: https://github.com/oscardegroot/lmpcc/blob/prius/lmpcc_solver/scripts/systems.py
        # w [-0.2, 0.2] | a [-1.0 1.0]
        # w was 0.5
        # delta was 0.45

        # NOTE: the angle of the vehicle should not be limited to -pi, pi, as the solution will not shift when it is at the border!
        # a was 3.0
        self.lower_bound = [-3.0, -1.5, -1.0e6, -1.0e6, -np.pi * 4, -0.01, -0.45, -1.0]
        self.upper_bound = [3.0, 1.5, 1.0e6, 1.0e6, np.pi * 4, 8.0, 0.45, 5000.0]

    def continuous_model(self, x, u):
        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]
        delta = x[4]

        wheel_base = 2.79  # between front wheel center and rear wheel center

        # NOTE: Mass is equally distributed according to the parameters
        lr = wheel_base / 2.0
        lf = wheel_base / 2.0
        ratio = lr / (lr + lf)

        beta = cd.arctan(ratio * cd.tan(delta))

        return np.array([v * cd.cos(psi + beta),
                        v * cd.sin(psi + beta),
                        (v / lr) * cd.sin(beta),
                        a, 
                        w])
    
    def model_discrete_dynamics(self, z, integrated_states):

        x = self.get_x()

        pos_x = x[0]
        pos_y = x[1]
        psi = x[2]
        vel = x[3]
        s = x[-1]

        # v = np.array([vel * cd.cos(psi), vel * cd.sin(psi)])

         # CA-MPCC
        path = Spline2D(self.params, self.settings["contouring"]["num_segments"], s)
        path_x, path_y = path.at(s)
        path_dx_normalized, path_dy_normalized = path.deriv_normalized(s)

        # Contour = n_vec
        contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
        # lag_error = -path_dx_normalized * (pos_x - path_x) - path_dy_normalized * (pos_y - path_y)

        # p_tilde = p + v * dt -> dp = p_tilde - p
        dp = np.array([integrated_states[0] - pos_x, integrated_states[1] - pos_y])
        t_vec = np.array([path_dx_normalized, path_dy_normalized])
        n_vec = np.array([path_dy_normalized, -path_dx_normalized])

        vt_t = dp.dot(t_vec)
        vn_t = dp.dot(n_vec)

        dt = self.settings["integrator_step"]

        R = 1. / path.get_curvature(s)
        R = cd.fmax(R, 1e5)

        theta = cd.atan2(vt_t, R - contour_error - vn_t)

        # Is vt vt_t / dt?
        # vt = vt_t / dt
        # vn = vn_t / dt

        # s_dot = R * (vt * (R - contour_error - vn_t) + vt * vn_t) / ((R - contour_error - vn_t)**2 + vt_t**2)

        return cd.vertcat(integrated_states, s + R*theta)


# Bicycle model with dynamic steering
# class BicycleModel2ndOrderWithDelay(DynamicModel):

#     def __init__(self, system):
#         self.nu = 2
#         self.nx = 7
#         super(BicycleModel2ndOrderWithDelay, self).__init__(system)

#         self.states = ['x', 'y', 'psi', 'v', 'delta', 'delta_in', 'spline']  # , 'ax', 'ay'
#         self.states_from_sensor = [True, True, True, True, False, True, False]  # , True, True
#         self.states_from_sensor_at_infeasible = [True, True, True, True, False, True, False]

#         self.inputs = ['a', 'w']
#         self.control_inputs['steering'] = 'delta_in'
#         self.control_inputs['velocity'] = 'v'
#         self.control_inputs['acceleration'] = 'a'
#         self.control_inputs['rot_velocity'] = 'w'

#     def continuous_model(self, x, u):
#         a = u[0]
#         w = u[1]
#         psi = x[2]
#         v = x[3]
#         delta = x[4] # affects the model
#         delta_in = x[5] # = w (i.e., is updated with the input)

#         ratio = self.system.ratio
#         lr = self.system.lr

#         beta = casadi.arctan(ratio * casadi.tan(delta))

#         return np.array([v * casadi.cos(psi + beta),
#                          v * casadi.sin(psi + beta),
#                          (v/lr) * casadi.sin(beta),
#                          a,
#                          0., # set in the discrete dynamics
#                          w,
#                          v])

# # Bicycle model with dynamic steering
# class BicycleModel2ndOrderWith2Delay(DynamicModel):

#     def __init__(self, system):
#         self.nu = 2
#         self.nx = 8
#         super(BicycleModel2ndOrderWith2Delay, self).__init__(system)

#         self.states = ['x', 'y', 'psi', 'v', 'delta', 'delta_in2', 'delta_in', 'spline']
#         self.states_from_sensor = [True, True, True, True, False, False, True, False]
#         self.states_from_sensor_at_infeasible = [True, True, True, True, False, False, True, False]

#         self.inputs = ['a', 'w']
#         self.control_inputs['steering'] = 'delta_in'
#         self.control_inputs['velocity'] = 'v'
#         self.control_inputs['acceleration'] = 'a'
#         self.control_inputs['rot_velocity'] = 'w'

#     def continuous_model(self, x, u):
#         a = u[0]
#         w = u[1]
#         psi = x[2]
#         v = x[3]
#         delta = x[4] # affects the model ( = delta_in2)
#         delta_in2 = x[5] # = delta_in
#         delta_in = x[6] # = w (i.e., is updated with the input)

#         ratio = self.system.ratio
#         lr = self.system.lr

#         beta = casadi.arctan(ratio * casadi.tan(delta))

#         return np.array([v * casadi.cos(psi + beta),
#                          v * casadi.sin(psi + beta),
#                          (v/lr) * casadi.sin(beta),
#                          a,
#                          0., # set in the discrete dynamics
#                          0., # set in the discrete dynamics
#                          w,
#                          v])
