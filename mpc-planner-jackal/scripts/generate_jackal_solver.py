import os
import sys

# TODO: Import packages through pypi
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-solver-generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-modules", "scripts"))

# SET YOUR FORCES PATH HERE (can also be in PYTHONPATH)
forces_path = os.path.join(os.path.expanduser("~"), "forces_pro_client")
sys.path.append(forces_path)

from util.files import load_settings
from control_modules import ModuleManager
from generate_solver import generate_solver

# Import modules here from mpc-planner-modules
from mpc_base import MPCBaseModule
from contouring import ContouringModule

from ellipsoid_constraints import EllipsoidConstraintModule
from guidance_constraints import GuidanceConstraintModule
from linearized_constraints import LinearizedConstraintModule

# Import solver models that you want to use
from solver_model import ContouringSecondOrderUnicycleModel


def define_modules(settings) -> ModuleManager:
    modules = ModuleManager()

    # Module that allows for penalization of variables
    base_module = modules.add_module(MPCBaseModule(settings))

    # Penalize ||a||_2^2 and ||w||_2^2
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")

    # Penalize ||v - v_ref||_2^2
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

generate_solver(modules, model, settings)
