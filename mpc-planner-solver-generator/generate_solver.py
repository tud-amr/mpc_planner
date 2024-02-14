
from util.files import load_settings, load_settings_path, write_to_yaml, solver_settings_path
from util.logging import print_value, print_success, print_header

from generate_cpp_files import generate_cpp_code

from solver_definition import define_modules

import solver_model

def generate_solver(modules=None, settings=None):
    if settings is None:
        settings = load_settings()
        
    if modules is None:
        modules = define_modules(settings)

    if settings["solver"] != "acados" and  settings["solver"] != "forces":
        raise IOError("Unknown solver specified in settings.yaml (should be 'acados' or 'forces')")

    print_header(f"Creating {settings['solver'].capitalize()} Solver: {settings['name']}_solver")

    # Define the model
    model = solver_model.ContouringSecondOrderUnicycleModel()

    if settings["solver"] == "forces":
        from generate_forces_solver import generate_forces_solver
        solver, simulator = generate_forces_solver(modules, settings, model)

    if settings["solver"] == "acados":
        from generate_acados_solver import generate_acados_solver
        solver, simulator = generate_acados_solver(settings, model)

    settings["params"].save_map()
    model.save_map()

    # Save other settings
    solver_settings = dict()
    solver_settings['N'] = settings["N"]
    solver_settings['nx'] = model.nx
    solver_settings['nu'] = model.nu
    solver_settings['nvar'] = model.get_nvar()
    solver_settings['npar'] = settings["params"].length()

    # Print settings
    # print_value("N", settings["N"], tab=True)
    # print_value("integrator step", settings["integrator_step"], tab=True)
    # print_success("Done!")

    path = solver_settings_path()
    write_to_yaml(path, solver_settings)

    generate_cpp_code(settings, model)

    return solver, simulator

if __name__ == "__main__":
    generate_solver()