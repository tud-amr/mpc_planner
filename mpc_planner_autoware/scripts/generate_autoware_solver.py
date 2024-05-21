import os
import sys

# TODO: Import packages through pypi
sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc_planner_modules", "scripts"))

# SET YOUR FORCES PATH HERE (can also be in PYTHONPATH)
forces_path = os.path.join(os.path.expanduser("~"), "forces_pro_client")
sys.path.append(forces_path)

from util.files import load_settings, get_current_package
from control_modules import ModuleManager
from generate_solver import generate_solver

# Import modules here from mpc_planner_modules
from mpc_base import MPCBaseModule
from contouring import ContouringModule
from path_reference_velocity import PathReferenceVelocityModule

from ellipsoid_constraints import EllipsoidConstraintModule
from gaussian_constraints import GaussianConstraintModule
from guidance_constraints import GuidanceConstraintModule
from linearized_constraints import LinearizedConstraintModule
from scenario_constraints import ScenarioConstraintModule
from contouring_constraints import ContouringConstraintModule

# Import solver models that you want to use
from solver_model import BicycleModel2ndOrder, BicycleModel2ndOrderCurvatureAware


# def configuration_safe_horizon(settings):
#     modules = ModuleManager()
#     model = ContouringSecondOrderUnicycleModelWithSlack()

#     # Module that allows for penalization of variables
#     base_module = modules.add_module(MPCBaseModule(settings))

#     # Penalize ||a||_2^2 and ||w||_2^2
#     base_module.weigh_variable(var_name="a", weight_names="acceleration")
#     base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
#     base_module.weigh_variable(var_name="slack", weight_names="slack")

#     # modules.add_module(GoalModule(settings))  # Track a goal
#     modules.add_module(ContouringModule(settings, num_segments=settings["contouring"]["num_segments"]))
#     modules.add_module(PathReferenceVelocityModule(settings, num_segments=settings["contouring"]["num_segments"]))

#     modules.add_module(ScenarioConstraintModule(settings))
#     return model, modules


def configuration_tmpc(settings):
    modules = ModuleManager()
    model = BicycleModel2ndOrderCurvatureAware()

    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    base_module.weigh_variable(var_name="slack", weight_names="slack")

    modules.add_module(ContouringModule(settings, 
        num_segments=settings["contouring"]["num_segments"], 
        preview=settings["contouring"]["preview"]))
    modules.add_module(PathReferenceVelocityModule(settings, num_segments=settings["contouring"]["num_segments"]))

    modules.add_module(ContouringConstraintModule(settings))
    modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=GaussianConstraintModule))
    # modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=EllipsoidConstraintModule))

    return model, modules


def configuration_lmpcc(settings):
    modules = ModuleManager()
    model = BicycleModel2ndOrderCurvatureAware()

    # Penalize ||a||_2^2 and ||w||_2^2
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    # base_module.weigh_variable(
    #     var_name="v",
    #     weight_names=["velocity", "reference_velocity"],
    #     cost_function=lambda x, w: w[0] * (x - w[1]) ** 2,
    # ) # Replaced with CA-MPC
    base_module.weigh_variable(var_name="slack", weight_names="slack")

    modules.add_module(ContouringModule(settings, 
                                        num_segments=settings["contouring"]["num_segments"],
                                        preview=settings["contouring"]["preview"]))
    # modules.add_module(PathReferenceVelocityModule(settings, num_segments=settings["contouring"]["num_segments"]))

    modules.add_module(ContouringConstraintModule(settings))
    modules.add_module(EllipsoidConstraintModule(settings))

    return model, modules


settings = load_settings()

model, modules = configuration_tmpc(settings)

generate_solver(modules, model, settings)
