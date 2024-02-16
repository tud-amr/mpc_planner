import os
import sys
import shutil

# SET YOUR FORCES PATH HERE
forces_path = os.path.join(os.path.expanduser("~"), "forces_pro_client")
sys.path.append(forces_path)
import forcespro.nlp

import numpy as np

from util.files import solver_name, solver_path, default_solver_path
from util.logging import print_value, print_success, print_header
from util.parameters import Parameters

import solver_model

from solver_definition import define_parameters, objective, constraints, constraint_upper_bounds, constraint_lower_bounds

# Press the green button in the gutter to run the script.
def generate_forces_solver(modules, settings, model):
    

    params = Parameters()
    define_parameters(modules, params, settings)
    settings["params"] = params

    modules.print()
    params.print()

    npar = params.length()
    nh = len(constraint_lower_bounds(settings))

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

        solver.objective[i] = objective_with_stage_index(i)

        # For all stages after the initial stage (k = 0)
        if i > 0:
            solver.ineq[i] = lambda z, p: constraints(z, p, model, settings)

            solver.nh[i] = nh

            solver.hu[i] = constraint_upper_bounds(settings)
            solver.hl[i] = constraint_lower_bounds(settings)
        else:
            solver.nh[i] = 0  # No constraints here

    # Equalities are specified on all stages
    solver.eq = lambda z, p: solver_model.discrete_dynamics(z, model, settings["integrator_step"])
    solver.E = np.concatenate([np.zeros((model.nx, model.nu)), np.eye(model.nx)], axis=1)

    # Initial stage (k = 0) specifies the states
    solver.xinitidx = range(model.nu, model.get_nvar())

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

    floating = True
    if floating:
        options.embedded_timing = 1
        options.license.use_floating_license = 1

    """
    PRIMAL DUAL INTERIOR POINT (Default Solver!) 
    """
    options.maxit = 500  # Maximum number of iterations
    options.mu0 = 20
    options.init = 0 # Warmstart with specified primal variables!

    # Creates code for symbolic model formulation given above, then contacts server to generate new solver
    print_header("Generating solver")
    generated_solver = solver.generate_solver(options)
    print_header("Output")
    
    if os.path.exists(solver_path(settings)) and os.path.isdir(solver_path(settings)): # Remove solver if it exists at the destination
        shutil.rmtree(solver_path(settings))
    shutil.move(default_solver_path(settings), solver_path(settings))  # Move the solver to this directory

    return generated_solver, generated_solver.dynamics    