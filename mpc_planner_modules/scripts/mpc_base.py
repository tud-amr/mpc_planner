import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

from control_modules import ObjectiveModule, Objective

"""
Weight optimization variables
"""


class WeightsObjective(Objective):

    def __init__(self, settings):
        super(WeightsObjective, self).__init__()

        self._weights = []

        self._weights_per_function = []
        self._variables_per_function = []
        self._cost_functions = []
        self._kwarg_list = []

    def define_parameters(self, params):
        for idx, param in enumerate(self._weights):
            params.add(param, add_to_rqt_reconfigure=True, **self._kwarg_list[idx])

        return params

    # Weights w are a parameter vector
    # Only add weights if they are not also parameters!
    def add(self, variable_to_weight, weight_names, cost_function=lambda x, w: w[0] * x**2, **kwargs):

        # # Make sure it's a list if it isn't yet
        if type(weight_names) != list:
            weight_names = [weight_names]

        # # Add all weights in the list
        for weight_name in weight_names:
            self._weights.append(weight_name)
            self._kwarg_list.append(kwargs)

        self._weights_per_function.append(weight_names)
        self._variables_per_function.append(variable_to_weight)
        self._cost_functions.append(cost_function)

    def get_value(self, model, params, settings, stage_idx):
        cost = 0.0
        for idx, cost_function in enumerate(self._cost_functions):
            weights = []
            for cost_weight in self._weights_per_function[idx]:  # Retrieve the weight parameters for this cost function!
                weights.append(params.get(cost_weight))

            variable = model.get(self._variables_per_function[idx])  # Retrieve the state / input to be weighted
            # _, _, var_range = model.get_bounds(self._variables_per_function[idx])

            # Add to the cost
            cost += cost_function(variable, weights)

        return cost

    def get_weights(self) -> list:
        return self._weights


class MPCBaseModule(ObjectiveModule):
    """
    Weight states and inputs of an MPC problem
    """

    def __init__(self, settings):
        super().__init__()
        self.module_name = "MPCBaseModule"  # Needs to correspond to the c++ name of the module
        self.import_name = "mpc_base.h"
        self.description = "Contains input and state penalties with weights that can be tuned in rqt_reconfigure"

        self.objectives.append(WeightsObjective(settings))

    # Add a variable that is weighted in the cost
    def weigh_variable(self, var_name, weight_names, **kwargs):
        self.objectives[0].add(var_name, weight_names, **kwargs)

    def add_definitions(self, header_file):
        weights = self.objectives[0].get_weights()

        header_file.write("#define WEIGHT_PARAMS {")

        for idx, weight in enumerate(weights):
            header_file.write('"' + weight + '"')
            if idx != len(weights) - 1:
                header_file.write(", ")
        header_file.write("}\n")
