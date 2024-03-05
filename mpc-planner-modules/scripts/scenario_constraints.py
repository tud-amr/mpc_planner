import os
import sys

import casadi as cd
import numpy as np

from control_modules import ConstraintModule
from linearized_constraints import LinearConstraints

sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc-planner-solver-generator"))


class ScenarioConstraintModule(ConstraintModule):
    """
    Linear constraints for scenario-based motion planning
    """

    def __init__(self, settings):
        super().__init__()
        self.module_name = "ScenarioConstraints"  # Needs to correspond to the c++ name of the module
        self.import_name = "scenario_constraints.h"
        self.dependencies.append("lmpcc_scenario_module")
        self.description = "Avoid dynamic obstacles under motion uncertainty using scenario optimization."

        self.constraints.append(LinearConstraints(n_discs=settings["n_discs"], n_constraints=24, use_slack=True))
