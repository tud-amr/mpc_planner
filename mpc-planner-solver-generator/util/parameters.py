import copy
import numpy as np

import casadi as cd # Acados

from util.files import write_to_yaml, parameter_map_path, load_settings
from util.logging import print_value, print_header



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