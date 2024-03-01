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
    cost = 0.0

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
def constraints(modules, z, p, model, settings, stage_idx):
    constraints = []

    params = settings["params"]
    params.load(p)
    model.load(z)

    for module in modules.modules:
        if module.type == "constraint":
            for constraint in module.constraints:
                constraints += constraint.get_constraints(model, params, settings, stage_idx)

    return constraints


def constraint_upper_bounds(modules):
    ub = []
    for module in modules.modules:
        if module.type == "constraint":
            for constraint in module.constraints:
                ub += constraint.get_upper_bound()
    return ub


def constraint_lower_bounds(modules):
    lb = []
    for module in modules.modules:
        if module.type == "constraint":
            for constraint in module.constraints:
                lb += constraint.get_lower_bound()
    return lb


def constraint_number(modules):
    nh = 0
    for module in modules.modules:
        if module.type == "constraint":
            for constraint in module.constraints:
                nh += constraint.nh
    return nh
