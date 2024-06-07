import os
import shutil


import forcespro.nlp

import numpy as np

from util.files import solver_name, solver_path, default_solver_path
from util.logging import print_warning, print_success, print_header
from util.parameters import Parameters

import solver_model

from solver_definition import (
    define_parameters,
    objective,
    constraints,
    constraint_lower_bounds,
    constraint_upper_bounds,
    constraint_number,
)


# Press the green button in the gutter to run the script.
def generate_forces_solver(modules, settings, model, skip_solver_generation):

    params = Parameters()
    define_parameters(modules, params, settings)
    settings["params"] = params
    solver_settings = settings["solver_settings"]["forces"]

    modules.print()
    params.print()

    npar = params.length()

    # Load model parameters from the settings
    solver = forcespro.nlp.SymbolicModel(settings["N"])
    solver.N = settings["N"]

    solver.nvar = model.get_nvar()  # number of online variables
    solver.neq = model.nx  # number of equality constraints
    solver.npar = npar

    # Bounds
    solver.lb = model.lower_bound
    solver.ub = model.upper_bound

    for i in range(0, solver.N):

        # Python cannot handle lambdas without an additional function that manually creates the lambda with the correct value
        def objective_with_stage_index(stage_idx):
            return lambda z, p: objective(modules, z, p, model, settings, stage_idx)

        def constraints_with_stage_index(stage_idx):
            return lambda z, p: constraints(modules, z, p, model, settings, stage_idx)

        solver.objective[i] = objective_with_stage_index(i)

        # For all stages after the initial stage (k = 0)
        if i > 0:
            solver.ineq[i] = constraints_with_stage_index(i)
            solver.hl[i] = constraint_lower_bounds(modules)
            solver.hu[i] = constraint_upper_bounds(modules)
            solver.nh[i] = constraint_number(modules)
        else:
            solver.nh[i] = 0  # No constraints here

    # Equalities are specified on all stages
    solver.eq = lambda z, p: model.discrete_dynamics(z, p, settings)  # solver_model.discrete_dynamics(z, p, model, settings)
    solver.E = np.concatenate([np.zeros((model.nx, model.nu)), np.eye(model.nx)], axis=1)

    # Initial stage (k = 0) specifies the states
    solver.xinitidx = model.get_xinit()  # range(model.nu, model.get_nvar())

    """
    Generate a solver
    """

    # Set solver options
    options = forcespro.CodeOptions(solver_name(settings))
    options.printlevel = 0  # 1 = timings, 2 = print progress
    options.optlevel = 3  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
    options.timing = 1
    options.overwrite = 1
    options.cleanup = 1

    if solver_settings["floating_license"]:
        options.embedded_timing = 1
        options.license.use_floating_license = 1

    multi_solver = True
    if multi_solver:
        options.threadSafeStorage = 1
        options.nlp.max_num_threads = 5

    if solver_settings["enable_timeout"]:
        options.solver_timeout = 1

    if not solver_settings["use_sqp"]:
        """
        PRIMAL DUAL INTERIOR POINT (Default Solver!)
        """
        options.maxit = 500  # Maximum number of iterations
        options.mu0 = 20
        options.init = solver_settings["init"]  # 0 = cold start, 1 = centerer start, 2 = warm start with the selected primal variables
        options.nlp.TolStat = settings["solver_settings"]["tolstat"]
    else:
        options.solvemethod = "SQP_NLP"
        options.sqp_nlp.maxqps = 1

        options.maxit = 100  # Maximum number of iterations
        options.sqp_nlp.qpinit = 0  # 1 # 1 = centered start, 0 = cold start

        options.nlp.linear_solver = "symm_indefinite_fast"
        options.sqp_nlp.reg_hessian = 5e-9  # = default

        options.exportBFGS = 1
        solver.bfgs_init = np.diag(np.array([0.301451, 0.50121, 0.206244, 1.08899, 0.0692479, 0.98321, 0.200556, 1]))

    # Creates code for symbolic model formulation given above, then contacts server to generate new solver
    if skip_solver_generation:
        print_header("Output")
        print_warning("Solver generation was disabled by the command line option. Skipped.", no_tab=True)
        return None, None
    else:
        print_header("Generating solver")
        generated_solver = solver.generate_solver(options)
        print_header("Output")

        if os.path.exists(solver_path(settings)) and os.path.isdir(solver_path(settings)):  # Remove solver if it exists at the destination
            shutil.rmtree(solver_path(settings))
        shutil.move(default_solver_path(settings), solver_path(settings))  # Move the solver to this directory

        return generated_solver, generated_solver.dynamics
