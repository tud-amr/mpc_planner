import copy
import numpy as np

import casadi as cd # Acados

from util.files import write_to_yaml, parameter_map_path, load_settings
from util.logging import print_value, print_header

class RealTimeParameters:

    def __init__(self, settings):
        self._map = load_settings("parameter_map")
        self._settings = settings

        self._num_p = self._map['num parameters']
        self._params = np.zeros((settings["N"], self._num_p))

    def set(self, k, parameter, value):
        if parameter in self._map.keys():
            self._params[k, self._map[parameter]] = value
            # print(f"{parameter} set to {value} | map value: {self._map[parameter]} check: {self._params[self._map[parameter]]}")

    def get(self, k, parameter):
        return self._params[k, self._map[parameter]]

    def get_solver_params(self):
        out = []
        for k in range(self._settings["N"]):
            for i in range(self._num_p):
                out.append(self._params[k, i])
        return out

    def get_solver_params_for_stage(self, k):
        out = []
        for i in range(self._num_p):
            out.append(self._params[k, i])
        return out

    def get_num_par(self):
        return self._num_p


class Parameters:

    def __init__(self):
        self._params = dict()
        self._param_idx = 0
        self._p = None

    def add(self, parameter):
        self._params[parameter] = copy.deepcopy(self._param_idx)
        self._param_idx += 1

    def length(self):
        return self._param_idx

    def load(self, p):
        self._p = p

    def save_map(self):
        file_path = parameter_map_path()

        map = self._params
        map['num parameters'] = self._param_idx
        write_to_yaml(file_path, self._params)

    def get(self, parameter):
        if self._p is None:
            print("Load parameters before requesting them!")

        return self._p[self._params[parameter]]

    def print(self):
        print_header("Parameters")
        print("----------")
        for param, idx in self._params.items():
            print_value(f"{idx}", f"{param}", tab=True)
        print("----------")

class AcadosParameters(Parameters):

    def __init__(self):
        super().__init__()

    def load_acados_parameters(self):

        self._p = []
        for param in self._params.keys():
            par = cd.SX.sym(param, 1)
            self._p.append(par)

        self.load(self._p)

    def get_acados_parameters(self):
        result = None
        for param in self._params.keys():
            if result is None:
                result = self.get(param)
            else:
                result = cd.vertcat(result, self.get(param))

        return result

    def get_acados_p(self):
        return self._p

class RealTimeModel:
    def __init__(self, settings, solver_settings):
        self._map = load_settings("model_map")
        self._settings = settings

        self._N = settings["N"]
        self._nu = solver_settings["nu"]
        self._nx = solver_settings["nx"]
        self._nvar = self._nu + self._nx
        self._vars = np.zeros((settings["N"], self._nvar))

    def get(self, k, var_name):
        map_value = self._map[var_name]
        return self._vars[k, map_value[1]]

class ForcesRealTimeModel(RealTimeModel):


    def __init__(self, settings, solver_settings):
        super().__init__(settings, solver_settings)

    def load(self, forces_output):
        for k in range(self._N):
            for var in range(self._nu):
                if k + 1< 10:
                    self._vars[k, var] = forces_output[f"x0{k+1}"][var]
                else:
                    self._vars[k, var] = forces_output[f"x{k+1}"][var]
            for var in range(self._nu, self._nvar):
                if k + 1< 10:
                    self._vars[k, var] = forces_output[f"x0{k+1}"][var - self._nu]
                else:
                    self._vars[k, var] = forces_output[f"x{k+1}"][var - self._nu]

    def get_trajectory(self, forces_output, mpc_x_plan, mpc_u_plan):

        for k in range(self._N):
            for i in range(self._nu):
                if k + 1 < 10:
                    mpc_u_plan[i, k] = forces_output[f"x0{k+1}"][i]
                else:
                    mpc_u_plan[i, k] = forces_output[f"x{k+1}"][i]
            for i in range(self._nu, self._nvar):
                if k + 1 < 10:
                    mpc_x_plan[i - self._nu, k] = forces_output[f"x0{k+1}"][i]
                else:
                    mpc_x_plan[i - self._nu, k] = forces_output[f"x{k+1}"][i]

        return np.concatenate([mpc_u_plan, mpc_x_plan])


class AcadosRealTimeModel(RealTimeModel):

    def __init__(self, settings, solver_settings):
        super().__init__(settings, solver_settings)

    def load(self, solver):
        # Load the solver data into a numpy array
        for k in range(self._settings["N"]):
            for var in range(self._nu):
                self._vars[k, var] = solver.get(k, 'u')[var]
            for var in range(self._nu, self._nvar):
                self._vars[k, var] = solver.get(k, 'x')[var - self._nu]

    def get_trajectory(self, solver, mpc_x_plan, mpc_u_plan):
        # Retrieve the trajectory
        for k in range(0, self._N):
            mpc_x_plan[:, k] = solver.get(k, 'x')
            mpc_u_plan[:, k] = solver.get(k, 'u')

        return np.concatenate([mpc_u_plan, mpc_x_plan])
