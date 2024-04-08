import os

import numpy as np
import casadi as cd

from acados_template import AcadosModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver

from util.files import solver_name, solver_path, default_solver_path
from util.logging import print_value, print_success, print_header, print_warning
from util.parameters import Parameters, AcadosParameters

from solver_definition import define_parameters, objective, constraints, constraint_lower_bounds, constraint_upper_bounds
import solver_model


def parse_constraint_bounds(bounds):
    large_value = 1e8  # Acados does not support inf
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
    ocp.constraints.lbx = np.array([model.lower_bound[model.nu : model.get_nvar()]]).flatten()
    ocp.constraints.ubx = np.array([model.upper_bound[model.nu : model.get_nvar()]]).flatten()
    ocp.constraints.idxbx = np.array(range(model.nx))

    # Set control input bound
    ocp.constraints.lbu = np.array([model.lower_bound[: model.nu]]).flatten()
    ocp.constraints.ubu = np.array([model.upper_bound[: model.nu]]).flatten()
    ocp.constraints.idxbu = np.array(range(model.nu))

    # Set path constraints bound
    nc = ocp.model.con_h_expr.shape[0]
    ocp.constraints.lh = parse_constraint_bounds(constraint_lower_bounds(modules))
    ocp.constraints.uh = parse_constraint_bounds(constraint_upper_bounds(modules))

    # Slack for constraints
    # ns = nc + nx
    # ocp.constraints.idxsh = np.array(range(nc))

    # Slack for state bounds
    ocp.constraints.idxsbx = np.array(range(nx))
    ocp.cost.zl = 1e2 * np.ones((model.nx,))
    ocp.cost.zu = 1e2 * np.ones((model.nx,))
    ocp.cost.Zl = 1e0 * np.ones((model.nx,))
    ocp.cost.Zu = 1e0 * np.ones((model.nx,))

    # ocp.constraints.idxsbx_e = np.array(range(nx))
    # ocp.constraints.idxsh_e = np.array([0])
    # ocp.cost.zl_e = 1e2 * np.ones(nx)
    # ocp.cost.zu_e = 1e2 * np.ones(nx)
    # ocp.cost.Zu_e = 1.0 * np.ones(nx)
    # ocp.cost.Zl_e = 1.0 * np.ones(nx)

    ocp.parameter_values = np.zeros(model_acados.p.size()[0])

    # horizon
    ocp.solver_options.tf = settings["N"] * settings["integrator_step"]
    ocp.solver_options.tol = 1e-3

    # Solver options
    # integrator option
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3

    # nlp solver options
    # ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # RTI gives out better solutions
    ocp.solver_options.nlp_solver_max_iter = 20
    ocp.solver_options.hessian_approx = "EXACT"

    # qp solver options
    # ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_iter_max = 10  # 100

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

    return solver, simulator


if __name__ == "__main__":
    generate_acados_solver()
