# MPC Planner - Solver Generator
To run the MPC in C++, this package generates an optimization solver with an interface for C++.

This package currently supports Forces Pro.

### Environment
Requirements for the Python solver generation are managed with Poetry. Use

```
poetry install
```

to set-up the environment. Note that `Python 3.8.10` is required to run this package (https://www.python.org/downloads/release/python-3810/).

### Usage
Below is an example on how to use this package to generate a solver. The solver below tracks a reference path and velocity, avoiding dynamic obstacles and penalizing acceleration and rotational velocity inputs.

```python
import sys, os
sys.path.append(os.path.join(sys.path[0],'..','..', 'mpc-planner-solver-generator'))
sys.path.append(os.path.join(sys.path[0],'..','..', 'mpc-planner-modules', 'scripts'))


from util.files import load_settings, default_solver_path, solver_path

from goal_module import GoalModule
from mpc_base import MPCBaseModule
from contouring import ContouringModule

from ellipsoid_constraints import EllipsoidConstraintModule

from solver_model import ContouringSecondOrderUnicycleModel
from control_modules import ModuleManager
from generate_solver import generate_solver

def define_modules(settings) -> ModuleManager:
    modules = ModuleManager()
    base_module = modules.add_module(MPCBaseModule(settings))  # Adds weights to the overall weight list
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    base_module.weigh_variable(var_name="v", weight_names=["velocity", "reference_velocity"], cost_function=lambda x, w, r: w[0] * (x - w[1]) ** 2 / r**2)
    
    modules.add_module(ContouringModule(settings, num_segments=settings["contouring"]["num_segments"]))  # Adds weights to the overall weight list

    modules.add_module(EllipsoidConstraintModule(settings)) 

    return modules

settings = load_settings()
modules = define_modules(settings)
model = ContouringSecondOrderUnicycleModel()

generate_solver(modules, model, settings)
```

To generate the solver, run:

```
poetry run python mpc-planner-<system>/generate_<system>_solver.py
```

A solver should be generated in `mpc-planner-solver/Solver` and mapping parameter files should be saved under `mpc-planner-solver/config`.

