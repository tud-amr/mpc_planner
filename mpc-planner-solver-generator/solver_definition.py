import numpy as np
import casadi as cd

def define_parameters(modules, params, settings):
    
    # Define parameters for objectives and constraints (in order)
    for module in modules.modules:
        if module.type == "objective":
            module.define_parameters(params)        
    
    for module in modules.modules:
        if module.type == "constraint":
            module.define_parameters(params)

    return params

def objective(modules, z, p, model, settings, stage_idx):
    cost = 0.

    params = settings["params"]
    params.load(p)
    model.load(z)
    
    for module in modules.modules:
        if module.type == "objective":
            cost += module.get_value(model, params, settings, stage_idx)
    
    # if stage_idx == 0:
        # print(cost)

    return cost

# lb <= constraints <= ub
# Ax <= b
# infinity <= Ax - b <= 0
def constraints(z, p, model, settings):
    constr = []
    
    if False:
        params = settings["params"]
        params.load(p)
        model.load(z)

        vehicle_pos = np.array([model.get('x'), model.get('y')])
        for m in range(settings["num_obstacles"]):
            A = np.array([params.get(f"obstacle{m}_A1"),
                        params.get(f"obstacle{m}_A2")])
            b = params.get(f"obstacle{m}_b")

            constr.append(A@vehicle_pos - b)

    return constr

def constraint_upper_bounds(settings):
    ub = []
    if False:
        for m in range(settings["num_obstacles"]):
            ub.append(0.)
    return ub

def constraint_lower_bounds(settings):
    lb = []
    if False:
        for m in range(settings["num_obstacles"]):
            lb.append(-np.inf)
    return lb