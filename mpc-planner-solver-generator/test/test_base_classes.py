import sys, os
sys.path.append(os.path.join(sys.path[0],'..'))

import numpy as np
import casadi

from util.parameters import Parameters
from util.files import load_settings, get_package_path, parameter_map_path, write_to_yaml
from util.logging import print_value, print_header, print_success, print_warning, print_path

import solver_model

def test_parameters():
    params = Parameters()

    params.add("var")
    params.add("v2")
    params.add("long variable name")

    params.print()

    assert params.length() == 3

    params.load([3.8, 2.5, -1.0])

    assert params.get("var") == 3.8
    assert params.get("v2") == 2.5
    assert params.get("long variable name") == -1.0

    # Load the current file
    temp = sys.argv[0]
    sys.argv[0] = os.path.join(get_package_path("mpc-planner-solver"), "src", "solver_interface.py")
    cur_file = load_settings("parameter_map")

    # Save the test 
    params.save_map()
    settings = load_settings("parameter_map") # Load the test
    assert settings["num parameters"] == 3

    # Restore the previous files
    file_path = parameter_map_path()
    write_to_yaml(file_path, cur_file)
    sys.argv[0] = temp

def test_model():
    model = solver_model.ContouringSecondOrderUnicycleModel()

    assert model.nx > 0
    assert model.nu > 0
    assert model.get_nvar() == model.nx + model.nu

    dx = model.continuous_model([0, 0, 0, 0, 0], [0, 0])
    assert dx.shape[0] == model.nx
    assert dx[0] == 0.
    assert dx[1] == 0.
    assert "x" in model.states
    assert "y" in model.states

    model.load([1., 2., 3., 4., 5., 6., 7.])
    assert model.get("x") == 3.
    assert model.get("y") == 4.
    assert model.get("a") == 1.

    try:
        a = model.get("xyz")
    except IOError:
        a = 1
    assert a == 1

    lb, ub, x_range = model.get_bounds("x")
    assert ub > lb
    assert x_range > 0

    res_cd = solver_model.numpy_to_casadi(np.array([1., 2., 3.]))

def test_logging():
    print_value("test", "value")
    print_value("test", 2)
    print_header("try")
    print_success("success")
    print_warning("warning")
    print_path("path", get_package_path("mpc-planner-solver"))