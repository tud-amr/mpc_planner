import copy
import numpy as np

import casadi as cd  # Acados

from util.files import write_to_yaml, parameter_map_path, load_settings
from util.logging import print_value, print_header


class Parameters:

    def __init__(self):
        self._params = dict()

        self.parameter_bundles = dict()  # Used to generate function names in C++ with an integer parameter

        self.rqt_params = []
        self.rqt_param_config_names = []
        self.rqt_param_min_values = []
        self.rqt_param_max_values = []

        self._param_idx = 0
        self._p = None

    def add(
        self,
        parameter,
        add_to_rqt_reconfigure=False,
        rqt_config_name=lambda p: f'["weights"]["{p}"]',
        bundle_name=None,
        rqt_min_value=0.0,
        rqt_max_value=100.0,
    ):
        """
        Adds a parameter to the parameter dictionary.

        Args:
            parameter (Any): The parameter to be added.
            add_to_rqt_reconfigure (bool, optional): Whether to add the parameter to the RQT Reconfigure. Defaults to False.
            rqt_config_name (function, optional): A function that returns the name of the parameter in CONFIG for the parameter in RQT Reconfigure. Defaults to lambda p: f'["weights"]["{p}"]'.
        """

        if parameter in self._params.keys():
            return

        self._params[parameter] = copy.deepcopy(self._param_idx)
        if bundle_name is None:
            bundle_name = parameter

        if bundle_name not in self.parameter_bundles.keys():
            self.parameter_bundles[bundle_name] = [copy.deepcopy(self._param_idx)]
        else:
            self.parameter_bundles[bundle_name].append(copy.deepcopy(self._param_idx))

        self._param_idx += 1

        if add_to_rqt_reconfigure:
            self.rqt_params.append(parameter)
            self.rqt_param_config_names.append(rqt_config_name)
            self.rqt_param_min_values.append(rqt_min_value)
            self.rqt_param_max_values.append(rqt_max_value)

    def length(self):
        return self._param_idx

    def load(self, p):
        self._p = p

    def save_map(self):
        file_path = parameter_map_path()

        map = self._params
        map["num parameters"] = self._param_idx
        write_to_yaml(file_path, self._params)

    def get_p(self) -> float:
        return self._p

    def get(self, parameter):
        if self._p is None:
            print("Load parameters before requesting them!")

        return self._p[self._params[parameter]]

    def print(self):
        print_header("Parameters")
        print("----------")
        for param, idx in self._params.items():
            if param in self.rqt_params:
                print_value(f"{idx}", f"{param} (in rqt_reconfigure)", tab=True)
            else:
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
