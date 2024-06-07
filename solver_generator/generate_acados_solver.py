import os, shutil

import numpy as np
import casadi as cd

from acados_template import AcadosModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver

from util.files import solver_name, solver_path, default_solver_path, default_acados_solver_path, acados_solver_path
from util.logging import print_value, print_success, print_header, print_warning, print_path
from util.parameters import Parameters, AcadosParameters

from solver_definition import define_parameters, objective, constraints, constraint_lower_bounds, constraint_upper_bounds
import solver_model


def parse_constraint_bounds(bounds):
    large_value = 1e15  # Acados does not support inf
    for i in range(len(bounds)):
        if bounds[i] == np.inf:
            bounds[i] = large_value
        if bounds[i] == -np.inf:
            bounds[i] = -large_value
    return np.array(bounds)


def create_acados_model(settings, model, modules):
    # Create an acados ocp model
    acados_model = AcadosModel()
    acados_model.name = solver_name(settings)

    # Dynamics
    z = model.acados_symbolics()
    dyn_f_expl, dyn_f_impl = model.get_acados_dynamics()

    # Parameters
    params = settings["params"]
    p = params.get_acados_p()

    # Constraints
    constr = cd.vertcat(*constraints(modules, z, p, model, settings, 1))

    if constr.shape[0] == 0:
        print("No constraints specified")
        constr = cd.SX()

    # stage cost
    cost_stage = objective(modules, z, p, model, settings, 1)

    # terminal cost
    cost_e = objective(modules, z, p, model, settings, settings["N"] - 1)

    # Formulating acados ocp model
    acados_model.x = model.get_x()
    acados_model.u = model.get_acados_u()
    acados_model.xdot = model.get_acados_x_dot()
    acados_model.f_expl_expr = dyn_f_expl
    acados_model.f_impl_expr = dyn_f_impl
    acados_model.p = params.get_acados_parameters()

    acados_model.cost_expr_ext_cost = cost_stage
    acados_model.cost_expr_ext_cost_e = cost_e
    acados_model.con_h_expr = constr

    return acados_model


def generate_acados_solver(modules, settings, model, skip_solver_generation):

    params = AcadosParameters()
    define_parameters(modules, params, settings)
    params.load_acados_parameters()
    settings["params"] = params
    solver_settings = settings["solver_settings"]

    modules.print()
    params.print()

    npar = params.length()

    model_acados = create_acados_model(settings, model, modules)

    # Create an acados ocp object
    ocp = AcadosOcp()
    ocp.model = model_acados

    # Set ocp dimensions
    ocp.dims.N = settings["N"]

    # Set cost types
    ocp.cost.cost_type = "EXTERNAL"
    # ocp.cost.cost_type_e = "EXTERNAL"

    # Set initial constraint
    ocp.constraints.x0 = np.zeros(model.nx)

    # Set state bound
    nx = model.nx
    nu = model.nu
    ocp.constraints.lbx = np.array([model.lower_bound[nu : model.get_nvar()]]).flatten()
    ocp.constraints.ubx = np.array([model.upper_bound[nu : model.get_nvar()]]).flatten()
    ocp.constraints.idxbx = np.array(range(model.nx))

    # Set control input bound
    ocp.constraints.lbu = np.array([model.lower_bound[:nu]]).flatten()
    ocp.constraints.ubu = np.array([model.upper_bound[:nu]]).flatten()
    ocp.constraints.idxbu = np.array(range(nu))

    # Set path constraints bound
    nc = ocp.model.con_h_expr.shape[0]
    ocp.constraints.lh = parse_constraint_bounds(constraint_lower_bounds(modules))
    ocp.constraints.uh = parse_constraint_bounds(constraint_upper_bounds(modules))

    # Slack for constraints
    add_constraint_slack = True
    value = 1.0e2  # 1.0e5

    ns = nx + nu
    if add_constraint_slack:
        ns += nc
        ocp.constraints.idxsh = np.array(range(nc))
        # ocp.constraints.idxsh_e = np.array(range(nc))

    ocp.constraints.idxsbx = np.array(range(nx))
    ocp.constraints.idxsbu = np.array(range(nu))

    # Slack for state bounds
    ocp.cost.zl = value * np.ones((ns,))
    ocp.cost.zu = value * np.ones((ns,))
    ocp.cost.Zl = value * np.ones((ns,))
    ocp.cost.Zu = value * np.ones((ns,))

    # ocp.constraints.idxsbx_e = np.array(range(nx))
    # ocp.cost.zl_e = value * np.ones(ns)
    # ocp.cost.zu_e = value * np.ones(ns)
    # ocp.cost.Zu_e = value * np.ones(ns)
    # ocp.cost.Zl_e = value * np.ones(ns)

    ocp.parameter_values = np.zeros(model_acados.p.size()[0])

    # horizon
    ocp.solver_options.tf = settings["N"] * settings["integrator_step"]
    ocp.solver_options.tol = 1e-2

    # Solver options
    # integrator option
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 5  # Number of divisions over the time horizon (ERK applied on each)

    # nlp solver options
    ocp.solver_options.nlp_solver_type = settings["solver_settings"]["acados"]["solver_type"]
    ocp.solver_options.nlp_solver_max_iter = 50
    ocp.solver_options.hessian_approx = "EXACT"
    # ocp.solver_options.levenberg_marquardt = 1e-1
    ocp.solver_options.globalization = "MERIT_BACKTRACKING"
    # ocp.solver_options.eps_sufficient_descent = 1e-1
    ocp.solver_options.qp_tol = 1e-3  # Important!

    # qp solver options
    # Full Condensing: Suitable for small to medium-sized systems, leading to a dense QP with only control inputs as decision variables.
    # Offers efficiency for smaller problems but faces scalability and memory issues for larger systems.
    # Partial Condensing: Suitable for larger systems, providing a balance between problem size and computational complexity.
    # It allows for controlled reduction in problem size, making it more scalable and flexible but potentially more complex to implement.
    # ocp.solver_options.qp_solver = "FULL_CONDENSING_QPOASES"
    # ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    # ocp.solver_options.qp_solver_iter_max = 100

    # code generation options
    ocp.code_export_directory = f"{os.path.dirname(os.path.abspath(__file__))}/acados/test"
    ocp.solver_options.print_level = 0

    # Generate the solver
    json_file_dir = f"{os.path.dirname(os.path.abspath(__file__))}/acados/{model_acados.name}/"
    json_file_name = json_file_dir + f"{model_acados.name}.json"
    os.makedirs(json_file_dir, exist_ok=True)

    if skip_solver_generation:
        print_header("Output")
        print_warning("Solver generation was disabled by the command line option. Skipped.", no_tab=True)
        return None, None
    else:
        print_header("Generating solver")
        solver = AcadosOcpSolver(acados_ocp=ocp, json_file=json_file_name)

        simulator = AcadosSimSolver(ocp, json_file=json_file_name)
        print_header("Output")

        if os.path.exists(acados_solver_path(settings)) and os.path.isdir(acados_solver_path(settings)):
            shutil.rmtree(acados_solver_path(settings))

        shutil.move(default_acados_solver_path(settings), acados_solver_path(settings))  # Move the solver to this directory

    return solver, simulator


if __name__ == "__main__":
    generate_acados_solver()
