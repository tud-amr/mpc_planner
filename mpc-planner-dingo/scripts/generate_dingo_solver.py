import os
import sys

import numpy as np

# TODO: Import packages through pypi
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-solver-generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-modules", "scripts"))

from util.files import load_settings

from mpc_base import MPCBaseModule
from contouring import ContouringModule

from ellipsoid_constraints import EllipsoidConstraintModule
from guidance_constraints import GuidanceConstraintModule
from linearized_constraints import LinearizedConstraintModule

from solver_model import ContouringSecondOrderUnicycleModel, ContouringPointMassModel
from control_modules import ModuleManager
from generate_solver import generate_solver


def define_modules(settings) -> ModuleManager:
    modules = ModuleManager()
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    base_module.weigh_variable(
        var_name="v",
        weight_names=["velocity", "reference_velocity"],
        cost_function=lambda x, w, r: w[0] * (x - w[1]) ** 2 / r**2,
    )

    # modules.add_module(GoalModule(settings))  # Track a goal
    modules.add_module(
        ContouringModule(settings, num_segments=settings["contouring"]["num_segments"])
    )

    # modules.add_module(EllipsoidConstraintModule(settings))
    modules.add_module(GuidanceConstraintModule(settings))
    # modules.add_module(LinearizedConstraintModule(settings))

    return modules


settings = load_settings()
modules = define_modules(settings)
model = ContouringSecondOrderUnicycleModel()
model.set_bounds(
    lower_bound=[-1.0, -2.0, -200.0, -200.0, -np.pi, -1.0, 0.0],
    upper_bound=[1.0, 4.0, 200.0, 200.0, np.pi, 1.3, 2000.0],
)

generate_solver(modules, model, settings)
