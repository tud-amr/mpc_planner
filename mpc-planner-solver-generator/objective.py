"""
Objective.py defines the objectives for the solver. They are included in ControlModules
DEPRECATED: Modules are now defined in mpc-planner-modules
"""

import numpy as np
import casadi

class Objective:
    
    def __init__(self) -> None:
        pass
    
    def define_parameters(self, params):
        raise IOError("Objective did not specify parameters")
    
    def get_value(self, model, params, settings, stage_idx) -> float:
        raise IOError("Objective did not return a cost")

    

# class SplineXY:

#     def __init__(self, param, spline_nr):

#         # Retrieve spline values from the parameters (stored as multi parameter by name)
#         self.x_a = param.get(f"spline{spline_nr}_ax")
#         self.x_b = param.get(f"spline{spline_nr}_bx")
#         self.x_c = param.get(f"spline{spline_nr}_cx")
#         self.x_d = param.get(f"spline{spline_nr}_dx")

#         self.y_a = param.get(f"spline{spline_nr}_ay")
#         self.y_b = param.get(f"spline{spline_nr}_by")
#         self.y_c = param.get(f"spline{spline_nr}_cy")
#         self.y_d = param.get(f"spline{spline_nr}_dy")
        
#         self.s_start = param.get(f"spline{spline_nr}_start")

#     def compute_path(self, spline_index):
#         self.path_x = self.x_a * (spline_index - self.s_start) ** 3 + \
#                       self.x_b * (spline_index - self.s_start) ** 2 + \
#                       self.x_c * (spline_index - self.s_start) + \
#                       self.x_d

#         self.path_y = self.y_a * (spline_index - self.s_start) ** 3 + \
#                       self.y_b * (spline_index - self.s_start) ** 2 + \
#                       self.y_c * (spline_index - self.s_start) + \
#                       self.y_d

#         self.path_dx = 3 * self.x_a * (spline_index - self.s_start) ** 2 + \
#                        2 * self.x_b * (spline_index - self.s_start) + \
#                        self.x_c

#         self.path_dy = 3 * self.y_a * (spline_index - self.s_start) ** 2 + \
#                        2 * self.y_b * (spline_index - self.s_start) + \
#                        self.y_c

# class ContouringObjective:

#     def __init__(self, settings, num_segments):

#         self.num_segments = num_segments

#     def define_parameters(self, params):
#         # Contour and lag are rqt parameters
        
#         params.add("contour")
#         params.add("lag")

#         for i in range(self.num_segments): # cubic spline in x and y plus their distance
#             params.add(f"spline{i}_start")
            
#             params.add(f"spline{i}_ax")
#             params.add(f"spline{i}_bx")
#             params.add(f"spline{i}_cx")
#             params.add(f"spline{i}_dx")            
            
#             params.add(f"spline{i}_ay")
#             params.add(f"spline{i}_by")
#             params.add(f"spline{i}_cy")
#             params.add(f"spline{i}_dy")
        
#         return params        

#     def get_value(self, model, params, settings, stage_idx):
#         cost = 0
        
#         pos_x = model.get('x')
#         pos_y = model.get('y')
#         s = model.get('spline')

#         contour_weight = params.get('contour')
#         lag_weight = params.get('lag')

#         splines = []  # Classes containing the splines
#         lambdas = []  # Merges splines
#         for i in range(self.num_segments):
#             splines.append(SplineXY(params, i))
#             splines[-1].compute_path(s)

#             # No lambda for the first segment (it is not glued to anything prior)
#             if i > 0:
#                 lambdas.append(1. / (1. + np.exp((s - splines[-1].s_start + 0.02) / 0.1)))  # Sigmoid

#         # We iteratively glue paths together here (start with the last path)
#         path_x = splines[-1].path_x
#         path_y = splines[-1].path_y
#         path_dx = splines[-1].path_dx
#         path_dy = splines[-1].path_dy
#         for k in range(len(splines) - 1, 0, -1):
#             # Glue with the previous path
#             path_x = lambdas[k - 1] * splines[k-1].path_x + (1. - lambdas[k - 1]) * path_x
#             path_y = lambdas[k - 1] * splines[k-1].path_y + (1. - lambdas[k - 1]) * path_y
#             path_dx = lambdas[k - 1] * splines[k-1].path_dx + (1. - lambdas[k - 1]) * path_dx
#             path_dy = lambdas[k - 1] * splines[k-1].path_dy + (1. - lambdas[k - 1]) * path_dy

#         path_norm = np.sqrt(path_dx ** 2 + path_dy ** 2)
#         path_dx_normalized = path_dx / path_norm
#         path_dy_normalized = path_dy / path_norm

#         contour_error = path_dy_normalized * (pos_x - path_x) - path_dx_normalized * (pos_y - path_y)
#         lag_error = -path_dx_normalized * (pos_x - path_x) - path_dy_normalized * (pos_y - path_y)

#         cost += contour_weight * contour_error ** 2
#         cost += lag_weight * lag_error ** 2

#         return cost


class PreviewObjective:

    def __init__(self, params, weight_list, n_segments, T):
        self.weight_list = weight_list

        self.T = T
        self.n_segments = n_segments

        self.define_parameters(params)

    def define_parameters(self, params):
        self.weight_list.append('preview')

    def get_value(self, z, model, settings, stage_idx):
        # Terminal state only
        if stage_idx < settings.N_bar - 2:
            return 0.

        cost = 0

        preview_weight = getattr(settings.params, 'preview')

        # Integrate the trajectory to obtain the preview point

        z_constant_input = casadi.vertcat(np.zeros((model.nu)), z[model.nu:model.nu+model.nx])

        z_preview = model.integrate(z_constant_input, self.T) # Integrate the dynamics T seconds ahead
        z_preview = casadi.vertcat(np.zeros((model.nu)), z_preview)

        pos_x = model.get_state(z_preview, 'x', True)
        pos_y = model.get_state(z_preview, 'y', True)
        s = model.get_state(z_preview, 'spline', True)

        spline = SplineParameters(settings.params, self.n_segments - 1) # Get the last spline
        spline.compute_path(s)

        path_norm = np.sqrt(spline.path_dx ** 2 + spline.path_dy ** 2)
        path_dx_normalized = spline.path_dx / path_norm
        path_dy_normalized = spline.path_dy / path_norm

        contour_error = path_dy_normalized * (pos_x - spline.path_x) - path_dx_normalized * (pos_y - spline.path_y)
        lag_error = -path_dx_normalized * (pos_x - spline.path_x) - path_dy_normalized * (pos_y - spline.path_y)

        cost += preview_weight * contour_error ** 2
        cost += preview_weight * lag_error ** 2
        # cost += settings.weights.lag * lag_error ** 2

        return cost



class ReferenceVelocityObjective:
    """
    Track a reference velocity that may change with time
    """

    def __init__(self, params, weight_list):
        self.weight_list = weight_list
        self.define_parameters(params)

    def define_parameters(self, params):
        params.add_parameter('velocity_reference') # Velocity reference is a parameter now!

        self.weight_list.append('velocity')

    def get_value(self, z, model, settings, stage_idx):

        # v = u[0] # Unicycle @todo
        v = model.get_state(z, 'v', True)
        velocity_reference = getattr(settings.params, 'velocity_reference')
        velocity_weight = getattr(settings.params, 'velocity')

        return velocity_weight * ((v - velocity_reference) ** 2)  # track the given velocity reference


class PathReferenceVelocityObjective:
    """
    Track a reference velocity along the path component that may change with time
    """

    def __init__(self, params, weight_list, n_segments):
        self.weight_list = weight_list
        self.define_parameters(params)
        self.n_segments = n_segments

    def define_parameters(self, params):
        params.add_parameter('velocity_reference') # Velocity reference is a parameter now!

        self.weight_list.append('velocity')

    def get_value(self, z, model, settings, stage_idx):

        # if stage_idx < settings.N: # @todo: Temporary for Khaled comparison!
        #     return 0.


        psi = model.get_state(z, 'psi', True)

        v = model.get_state(z, 'v', True)
        s = model.get_state(z, 'spline', True)

        velocity_reference = getattr(settings.params, 'velocity_reference')
        velocity_weight = getattr(settings.params, 'velocity')

        # return velocity_weight * (v - velocity_reference)**2 #  @todo: Temporary for Khaled comparison!

        # @todo: Put this somewhere general (now copied from contouring weights)
        splines = []  # Classes containing the splines
        lambdas = []  # Merges splines
        for i in range(self.n_segments):
            splines.append(SplineParameters(settings.params, i))
            splines[-1].compute_path(s)

            # No lambda for the first segment (it is not glued to anything prior)
            if i > 0:
                lambdas.append(1. / (1. + np.exp((s - splines[-1].s_start + 0.02) / 0.1)))  # Sigmoid

        # We iteratively glue paths together here (start with the last path)
        path_x = splines[-1].path_x
        path_y = splines[-1].path_y
        path_dx = splines[-1].path_dx
        path_dy = splines[-1].path_dy
        for k in range(len(splines) - 1, 0, -1):
            # Glue with the previous path
            # path_x = lambdas[k - 1] * splines[k - 1].path_x + (1. - lambdas[k - 1]) * path_x
            # path_y = lambdas[k - 1] * splines[k - 1].path_y + (1. - lambdas[k - 1]) * path_y
            path_dx = lambdas[k - 1] * splines[k - 1].path_dx + (1. - lambdas[k - 1]) * path_dx
            path_dy = lambdas[k - 1] * splines[k - 1].path_dy + (1. - lambdas[k - 1]) * path_dy

        path_norm = np.sqrt(path_dx ** 2 + path_dy ** 2)
        path_dx_normalized = path_dx / path_norm
        path_dy_normalized = path_dy / path_norm

        path_angle = casadi.atan2(path_dy_normalized, path_dx_normalized)
        diff_angle = path_angle - psi
        v_path = casadi.cos(diff_angle) * v

        return velocity_weight * ((v_path - velocity_reference) ** 2)  # track the given velocity reference


def objective(z, param, model, settings, stage_idx):
    # initialise cost at 0
    cost = 0.0
    settings.params.load_params(param)

    # Objective modules
    for module in settings.modules.modules:
        if module.type == "objective":
            for module_objective in module.objectives:
                cost += module_objective.get_value(z, model, settings, stage_idx)

    return cost