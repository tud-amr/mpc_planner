import os
import sys

# TODO: Import packages through pypi
sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc_planner_modules", "scripts"))

# SET YOUR FORCES PATH HERE (can also be in PYTHONPATH)
forces_path = os.path.join(os.path.expanduser("~"), "forces_pro_client")
sys.path.append(forces_path)

from util.files import load_settings
from control_modules import ModuleManager
from generate_solver import generate_solver

# Import modules here from mpc_planner_modules
from mpc_base import MPCBaseModule
from contouring import ContouringModule
from goal_module import GoalModule
from path_reference_velocity import PathReferenceVelocityModule

from ellipsoid_constraints import EllipsoidConstraintModule
from gaussian_constraints import GaussianConstraintModule
from guidance_constraints import GuidanceConstraintModule
from linearized_constraints import LinearizedConstraintModule

# Import solver models that you want to use
from solver_model import ContouringSecondOrderUnicycleModel


def configuration_goal_tmpc(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModel()

    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    # Penalize ||v - v_ref||_2^2
    base_module.weigh_variable(
        var_name="v",
        weight_names=["velocity", "reference_velocity"],
        cost_function=lambda x, w: w[0] * (x - w[1]) ** 2,
    )

    modules.add_module(GoalModule(settings))
    modules.add_module(EllipsoidConstraintModule(settings))

    # modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=EllipsoidConstraintModule))

    return model, modules


def configuration_tmpc(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModel()

    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")

    modules.add_module(ContouringModule(settings, num_segments=settings["contouring"]["num_segments"]))
    modules.add_module(PathReferenceVelocityModule(settings, num_segments=settings["contouring"]["num_segments"]))

    # modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=EllipsoidConstraintModule))
    modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=GaussianConstraintModule))

    return model, modules


def configuration_lmpcc(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModel()

    # Penalize ||a||_2^2 and ||w||_2^2
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")

    modules.add_module(ContouringModule(settings, num_segments=settings["contouring"]["num_segments"]))
    modules.add_module(PathReferenceVelocityModule(settings, num_segments=settings["contouring"]["num_segments"]))

    modules.add_module(EllipsoidConstraintModule(settings))

    return model, modules


settings = load_settings()

model, modules = configuration_tmpc(settings)

generate_solver(modules, model, settings)
