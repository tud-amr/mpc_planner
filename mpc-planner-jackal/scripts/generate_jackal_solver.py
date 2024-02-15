import numpy as np
import casadi as cd

import shutil

# TODO set up packages properly
import sys, os
sys.path.append(os.path.join(sys.path[0],'..','..', 'mpc-planner-solver-generator'))
sys.path.append(os.path.join(sys.path[0],'..','..', 'mpc-planner-modules', 'scripts'))

from util.files import load_settings, default_solver_path, solver_path
from util.logging import print_value, print_path, print_success

from goal_module import GoalModule
from mpc_base import MPCBaseModule
from contouring import ContouringModule

from control_modules import ModuleManager
from generate_solver import generate_solver


def define_modules(settings) -> ModuleManager:
    modules = ModuleManager(settings)
    base_module = modules.add_module(MPCBaseModule(settings))  # Adds weights to the overall weight list
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    base_module.weigh_variable(var_name="v", weight_names=["velocity", "reference_velocity"], cost_function=lambda x, w, r: w[0] * (x - w[1]) ** 2 / r**2)
    
    # modules.add_module(GoalModule(settings))  # Track a goal
    modules.add_module(ContouringModule(settings, num_segments=5))  # Adds weights to the overall weight list

    return modules

settings = load_settings()
modules = define_modules(settings)

generate_solver(modules, settings)

print_path("Solver path", solver_path(settings), tab=True, end="")
print_success(" -> generated")

