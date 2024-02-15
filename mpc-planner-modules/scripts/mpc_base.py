import sys, os
sys.path.append(os.path.join(sys.path[0],'..','..', 'mpc-planner-solver-generator'))

from control_modules import ObjectiveModule
from objective import Objective

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

    def define_parameters(self, params):
        for param in self._weights:
            params.add(param)

        return params
    
    # Weights w are a parameter vector
    # Only add weights if they are not also parameters!
    def add(self, variable_to_weight, weight_names, cost_function=lambda x, w: w[0] * x ** 2):

        # # Make sure it's a list if it isn't yet
        if type(weight_names) != list:
            weight_names = [weight_names]

        # # Add all weights in the list
        for weight_name in weight_names:
            self._weights.append(weight_name)

        self._weights_per_function.append(weight_names)
        self._variables_per_function.append(variable_to_weight)
        self._cost_functions.append(cost_function)

    def get_value(self, model, params, settings, stage_idx):
        cost = 0.0
        for idx, cost_function in enumerate(self._cost_functions):
            weights = []
            for cost_weight in self._weights_per_function[idx]: # Retrieve the weight parameters for this cost function!
                weights.append(params.get(cost_weight))

            variable = model.get(self._variables_per_function[idx])  # Retrieve the state / input to be weighted
            _, _, var_range = model.get_bounds(self._variables_per_function[idx])
            variable = variable / var_range  # Normalize the variable

            # Add to the cost
            cost += cost_function(variable, weights)

        return cost


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