import sys, os
sys.path.append("../../../mpc_planner_analysis/")

import shutil

# import helpers
# data_folder, metric_folder, figure_folder = helpers.get_folder_names(PROJECT_NAME)


import scripts.metrics as metrics
import json, yaml
# from load_ros_data import *

import optuna

from optuna.visualization import plot_optimization_history
from optuna.visualization import plot_param_importances

import subprocess

class CostFunction:
       
        def __init__(self):
                self.costs = dict()
                self.cost_contributions = dict()

        def add_cost(self, name, weight, normalization=1., offset=0.):
                if offset > 0.:
                        self.costs[name] = lambda x: weight * ((offset - x) / normalization)
                else:
                        self.costs[name] = lambda x: weight * (x / normalization)

        def compute_cost(self, name, value):
                self.cost_contributions[name] = self.costs[name](value)

        def get_cost(self):
                cost = 0.
                for _, value in self.cost_contributions.items():
                        cost += value
                return cost

        def print(self):
                for key, value in self.cost_contributions.items():
                        print(f"Cost value for {key}: {value}")
                print("-------")
                print("total: " + str(self.get_cost()))
        
def configuration_autoware():

        settings = dict()
        settings["num_trials"] = 50
        settings["num_experiments"] = 3
        settings["tune_guidance"] = False
        settings["tune_terminal"] = False

        settings["config_path"] = os.path.dirname(os.path.realpath(__file__)) + "/../config/"
        settings["settings_path"] = settings["config_path"] + "settings.yaml"
        settings["settings_backup_path"] = settings["config_path"] + "backup_settings.yaml"
        settings["guidance_planner_settings_path"] = settings["config_path"] + "autotune_guidance.yaml"

        settings["project_folder"] = os.path.dirname(os.path.realpath(__file__)) + "/../../../mpc_planner_analysis/results/"
        settings["data_folder"] = settings["project_folder"] + "data"

        return settings


def define_parameters(trial):
    
        settings = configuration_autoware()

        with open(settings["settings_path"], 'r') as file:
                params = yaml.safe_load(file)

        print("Loaded settings from: " + settings["settings_path"])
        
        # Copy the original settings
        shutil.copy(settings["settings_path"], settings["settings_backup_path"])

        # Modify the settings
        # Non-tunable settings!
        params["recording"]["enable"] = True
        params["recording"]["num_experiments"] = settings["num_experiments"]
        params["recording"]["folder"] = settings["data_folder"]
        params["recording"]["file"] = "autotune_TMPC"

        # MPC Planner Weights
        params["weights"]["acceleration"] = trial.suggest_float("weights/acceleration", 0.1, 20.0)
        params["weights"]["angular_velocity"] = trial.suggest_float("weights/angular_velocity", 0.1, 20.0)
        params["weights"]["contour"] = trial.suggest_float("weights/contour", 0.1, 10.0)

        if settings["tune_terminal"]:
                params["weights"]["terminal_angle"] = trial.suggest_float("weights/terminal_angle", 0.1, 10.0)
                params["weights"]["terminal_contouring"] = trial.suggest_float("weights/terminal_contouring", 0.1, 10.0)

        # params["weights"]["velocity"] = trial.suggest_float("weights/velocity", 0.1, 10.0)
        # params["weights"]["slack"] = trial.suggest_float("weights/slack", 1000, 10000)

        # Guidance planner
        if settings["tune_guidance"]:
                guidance_params = dict()
                guidance_params["/**"] = dict()
                guidance_params["/**"]["ros__parameters"] = dict()
                ros_guidance_params = guidance_params["/**"]["ros__parameters"]

                ros_guidance_params["prm"] = dict()
                ros_guidance_params["prm"]["n_samples"] = trial.suggest_int("prm/n_samples", 10, 60)
                ros_guidance_params["prm"]["max_acceleration"] = trial.suggest_float("prm/max_acceleration", 3.0, 4.0)
                ros_guidance_params["prm"]["max_velocity"] = trial.suggest_float("prm/max_velocity", 3.0, 4.0)
                ros_guidance_params["prm"]["goals"] = dict()
                ros_guidance_params["prm"]["goals"]["longitudinal"] = trial.suggest_int("prm/goals/longitudinal", 3, 5)
                # ros_guidance_params["prm"]["topology_comparison"] = trial.suggest_categorical(
                        # "prm/topology_comparison", 
                        # ["Homology", "Winding"])
                
                write_to_yaml(settings["guidance_planner_settings_path"], guidance_params)

        trial.set_user_attr("settings", settings)

        print("Trial settings ready")

        return params

def restore_parameters(trial):
        print("Restoring original settings")
        settings = trial.user_attrs["settings"]
        shutil.copy(settings["settings_backup_path"], settings["settings_path"])
        os.remove(settings["settings_backup_path"])

def write_to_yaml(path, params):

    if not os.path.exists(path):
        os.mknod(path)

    with open(path, "w") as outfile:
        yaml.dump(params, outfile, default_flow_style=False)

def evaluate(trial):
        settings = trial.user_attrs["settings"]

        m = metrics.main("autotune", "TMPC", settings["project_folder"], remove_first=False, verbose=True)

        cost = CostFunction()
        cost.add_cost("Infeasibility", 0.5)
        cost.add_cost("Collision", 5.)
        cost.add_cost("Duration", -10., normalization=50., offset=50.)
        cost.add_cost("Runtime", -1., normalization=100., offset=100.)
        cost.add_cost("Acceleration", -1., normalization=80.)

        cost.compute_cost("Infeasibility", float(sum(m["num infeasible"])))
        cost.compute_cost("Collision", float(sum(m["collisions"])))
        cost.compute_cost("Duration", m["task duration"]["mean"])
        cost.compute_cost("Runtime", m["runtime"]["mean"] * 1000.)
        cost.compute_cost("Acceleration", m["acceleration"]["max"])

        cost.print()

        return cost.get_cost()


def run_scenario(trial):

        settings = trial.user_attrs["settings"]

        source_path = f"/workspaces/autoware_iv/install/setup.bash"
        source_command = f"source {source_path}"

        run_command = f"{source_command} && ros2 launch mpc_planner_autoware ros2_autoware.launch standalone_mode:=true"
        print("Running command: " + run_command)
        try:
                subprocess.call(["bash", "-c", run_command], timeout=60*settings["num_experiments"])
        except subprocess.TimeoutExpired:
               pass

def autotune_trial(trial):

        params = define_parameters(trial)
        write_to_yaml(settings["settings_path"], params)

        try:
                # Run the planner scenario
                run_scenario(trial)

                # Load metrics and compute the cost
                cost = evaluate(trial)

        except Exception as e:
                restore_parameters(trial)
                raise IOError(f"Trial Failed: {e}")

        # trial.report(cost, 0)
        # if trial.should_prune():
                # raise optuna.TrialPruned()

        restore_parameters(trial)

        return cost


if __name__ == '__main__':
        study_path = os.path.dirname(os.path.realpath(__file__)) + "/../studies/"
        os.makedirs(study_path, exist_ok=True)

        settings = configuration_autoware()

        # If load do not delete
        load_study = True
        delete_study = not load_study
        skip_optimization = False
        n_trials = settings["num_trials"]
        study_name = "initial.db"

        full_study_name = study_path + "/" + study_name

        print("New study: " + full_study_name)

        # if delete_study:
        # inp = input("Are you sure you want to delete the " + study_name + " study? (y/n) ")
        # if inp == 'y':
        #         optuna.delete_study(storage="sqlite:///" + study_name,
        #                         study_name=study_name)
        #         print("Deleted study: " + study_name)
        # else:
        #         raise IOError("Please set delete_study to False")

        if delete_study:
                print("deleting previous study")
                try:
                        optuna.delete_study(storage="sqlite:///" + study_name, study_name=study_name)
                except:
                        print("No previous study found")

        study = optuna.create_study(direction="minimize",
                                storage="sqlite:///" + study_name, #+ ".db",
                                study_name=study_name,
                                pruner=optuna.pruners.MedianPruner(),
                                load_if_exists=load_study)

        if not skip_optimization:
                study.optimize(autotune_trial, n_trials=n_trials)

        print(study.best_trial)

        # plot_param_importances(study).show()
        # plot_optimization_history(study).show()

        # # Leave the autotune parameters in the best trial state
        # params = define_parameters(study.best_trial)
        # write_to_yaml(params)

