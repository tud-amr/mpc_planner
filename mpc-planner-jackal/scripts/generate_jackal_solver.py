import numpy as np
import casadi as cd

import shutil

# TODO set up packages properly
import sys, os
sys.path.append(os.path.join(sys.path[0],'..','..', 'mpc-planner-solver-generator'))

from util.files import load_settings, default_solver_path, solver_path
from util.logging import print_value

from control_modules import MPCBaseModule, ContouringModule, ModuleManager
from generate_solver import generate_solver


def define_modules(settings) -> ModuleManager:
    modules = ModuleManager(settings)
    base_module = modules.add_module(MPCBaseModule(settings))  # Adds weights to the overall weight list
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    
    # modules.add_module(ContouringModule(settings, num_segments=5))  # Adds weights to the overall weight list

    return modules

settings = load_settings()
modules = define_modules(settings)

generate_solver(modules, settings)

print_value("Solver generated at", solver_path(settings), tab=True)

