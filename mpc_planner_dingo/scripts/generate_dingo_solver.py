import os
import sys

import numpy as np

# TODO: Import packages through pypi
sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc_planner_modules", "scripts"))

# SET YOUR FORCES PATH HERE (can also be in PYTHONPATH)
forces_path = os.path.join(os.path.expanduser("~"), "forces_pro_client")
sys.path.append(forces_path)

from util.files import load_settings

from mpc_base import MPCBaseModule
from contouring import ContouringModule

from gaussian_constraints import GaussianConstraintModule
from ellipsoid_constraints import EllipsoidConstraintModule
from guidance_constraints import GuidanceConstraintModule
from linearized_constraints import LinearizedConstraintModule
from path_reference_velocity import PathReferenceVelocityModule
from goal_module import GoalModule

from solver_model import DynamicsModel, ContouringSecondOrderUnicycleModel
from control_modules import ModuleManager
from generate_solver import generate_solver


class ContouringPointMassModel(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2
        self.nx = 4

        self.states = ["x", "y", "vx", "vy"]
        self.inputs = ["ax", "ay"]

        self.lower_bound = [-1.0, -1.0, -200.0, -200.0, -1.0, -1.0]
        self.upper_bound = [1.0, 1.0, 200.0, 200.0, 1.0, 1.0]

    def continuous_model(self, x, u):
        return np.array([x[2], x[3], u[0], u[1]])


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
    modules.add_module(ContouringModule(settings, num_segments=settings["contouring"]["num_segments"]))

    # modules.add_module(EllipsoidConstraintModule(settings))
    modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=GaussianConstraintModule))
    # modules.add_module(LinearizedConstraintModule(settings))
    # modules.add_module(GaussianConstraintModule(settings))

    return modules


def configuration_lmpcc(settings):
    modules = ModuleManager()
    model = ContouringPointMassModel()
    # model = ContouringSecondOrderUnicycleModel()
    # model.set_bounds(
    #     lower_bound=[-1.0, -2.0, -200.0, -200.0, -np.pi, -1.0, 0.0],
    #     upper_bound=[1.0, 4.0, 200.0, 200.0, np.pi, 1.3, 2000.0],
    # )

    # Penalize ||a||_2^2 and ||w||_2^2
    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="ax", weight_names="acceleration_x")
    base_module.weigh_variable(var_name="ay", weight_names="acceleration_y")

    modules.add_module(GoalModule(settings))
    # modules.add_module(ContouringModule(settings, num_segments=settings["contouring"]["num_segments"]))
    # modules.add_module(PathReferenceVelocityModule(settings, num_segments=settings["contouring"]["num_segments"]))
    modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=EllipsoidConstraintModule))

    # modules.add_module(EllipsoidConstraintModule(settings))

    return model, modules


settings = load_settings()

model, modules = configuration_lmpcc(settings)

generate_solver(modules, model, settings)
