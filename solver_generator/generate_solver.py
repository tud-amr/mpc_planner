import sys

from util.files import load_settings, write_to_yaml
from util.files import solver_path, solver_settings_path

from util.logging import print_success, print_header, print_path

from generate_cpp_files import generate_cpp_code, generate_module_header, generate_module_cmake
from generate_cpp_files import generate_module_definitions, generate_rqtreconfigure, generate_module_packagexml
from generate_cpp_files import generate_ros2_rqtreconfigure


def generate_solver(modules, model, settings=None):
    skip_solver_generation = len(sys.argv) > 1 and sys.argv[1].lower() == "false"

    if settings is None:
        settings = load_settings()

    if settings["solver_settings"]["solver"] != "acados" and settings["solver_settings"]["solver"] != "forces":
        raise IOError("Unknown solver specified in settings.yaml" "(should be 'acados' or 'forces')")

    print_header(f"Creating {settings['solver_settings']['solver'].capitalize()}" f"Solver: {settings['name']}_solver")

    if settings["solver_settings"]["solver"] == "forces":
        from generate_forces_solver import generate_forces_solver

        solver, simulator = generate_forces_solver(modules, settings, model, skip_solver_generation)

    if settings["solver_settings"]["solver"] == "acados":
        from generate_acados_solver import generate_acados_solver

        solver, simulator = generate_acados_solver(settings, model)

    settings["params"].save_map()
    model.save_map()

    # Save other settings
    solver_settings = dict()
    solver_settings["N"] = settings["N"]
    solver_settings["nx"] = model.nx
    solver_settings["nu"] = model.nu
    solver_settings["nvar"] = model.get_nvar()
    solver_settings["npar"] = settings["params"].length()

    path = solver_settings_path()
    write_to_yaml(path, solver_settings)

    generate_cpp_code(settings, model)
    generate_module_header(modules)
    generate_module_definitions(modules)
    generate_module_cmake(modules)
    generate_module_packagexml(modules)
    generate_rqtreconfigure(settings)
    generate_ros2_rqtreconfigure(settings)
    
    print_path("Solver", solver_path(settings), tab=True, end="")
    print_success(" -> generated")

    return solver, simulator


if __name__ == "__main__":
    generate_solver()
