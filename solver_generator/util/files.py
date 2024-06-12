import os, sys
import yaml

from util.logging import print_success, print_value, print_path


def get_base_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))


def get_current_package():
    return os.path.dirname(os.path.realpath(sys.argv[0])).split("/")[-2]


def get_package_path(package_name):
    return os.path.join(os.path.dirname(__file__), f"../../{package_name}")


def get_solver_package_path():
    return get_package_path("mpc_planner_solver")


def save_config_path():
    config_path = os.path.join(get_solver_package_path(), "config/")
    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    return config_path


def load_config_path():
    return os.path.join(get_base_path(), "../config")


def load_settings_path(setting_file_name="settings"):
    return os.path.join(load_config_path(), f"{setting_file_name}.yaml")


def load_settings(setting_file_name="settings"):
    path = load_settings_path(setting_file_name)
    print_path("Settings", path, end="")
    with open(path, "r") as stream:
        settings = yaml.safe_load(stream)
    print_success(f" -> loaded")
    return settings


def load_test_settings(setting_file_name="settings"):
    path = f"{get_package_path('mpc_planner_jackalsimulator')}/config/{setting_file_name}.yaml"
    print_path("Settings", path, end="")
    with open(path, "r") as stream:
        settings = yaml.safe_load(stream)
    print_success(f" -> loaded")
    return settings


def default_solver_path(settings):
    return os.path.join(os.getcwd(), f"{solver_name(settings)}")


def solver_path(settings):
    return os.path.join(get_solver_package_path(), f"{solver_name(settings)}")


def default_acados_solver_path(settings):
    return os.path.join(get_package_path("solver_generator"), f"acados")


def acados_solver_path(settings):
    return os.path.join(get_solver_package_path(), f"acados")


def parameter_map_path():
    return os.path.join(save_config_path(), f"parameter_map.yaml")


def model_map_path():
    return os.path.join(save_config_path(), f"model_map.yaml")


def solver_settings_path():
    return os.path.join(save_config_path(), f"solver_settings.yaml")


def generated_src_file(settings):
    return os.path.join(solver_path(settings), f"mpc_planner_generated.cpp")


def planner_path():
    return get_package_path("mpc_planner")


def generated_include_file(settings):
    include_path = os.path.join(solver_path(settings), f"include/")
    os.makedirs(include_path, exist_ok=True)
    print_path("Generated Header", f"{include_path}mpc_planner_generated.h/cpp", tab=True, end="")
    return f"{include_path}mpc_planner_generated.h", f"{include_path}mpc_planner_generated.cpp"


def generated_parameter_include_file(settings):
    include_path = os.path.join(get_package_path("mpc_planner_solver"), f"include/mpc_planner_solver/")
    src_path = os.path.join(get_package_path("mpc_planner_solver"), f"src/")
    os.makedirs(include_path, exist_ok=True)
    print_path("Generated Parameter Header", f"{include_path}mpc_planner_parameters.h/cpp", tab=True, end="")
    return f"{include_path}mpc_planner_parameters.h", f"{src_path}mpc_planner_parameters.cpp"


def solver_name(settings):
    return "Solver"


def write_to_yaml(filename, data):
    with open(filename, "w") as outfile:
        yaml.dump(data, outfile, default_flow_style=False)
