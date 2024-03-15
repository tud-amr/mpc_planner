import sys, os

sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

from control_modules import ObjectiveModule, Objective

"""
Track a goal in 2D
"""


class GoalObjective(Objective):

    def __init__(self, settings):
        pass

    def define_parameters(self, params):
        params.add("goal_weight")
        params.add("goal_x")
        params.add("goal_y")

    def get_value(self, model, params, settings, stage_idx):
        cost = 0

        if stage_idx == settings["N"] - 1:
            print(stage_idx)
            pos_x = model.get("x")
            pos_y = model.get("y")

            goal_weight = params.get("goal_weight")

            goal_x = params.get("goal_x")
            goal_y = params.get("goal_y")

            cost += goal_weight * ((pos_x - goal_x) ** 2 + (pos_y - goal_y) ** 2) / (goal_x**2 + goal_y**2 + 0.01)

        return cost


class GoalModule(ObjectiveModule):

    def __init__(self, settings):
        super().__init__()
        self.module_name = "GoalModule"  # Needs to correspond to the c++ name of the module
        self.import_name = "goal_module.h"
        self.description = "Tracks a goal in 2D"

        self.objectives.append(GoalObjective(settings))
