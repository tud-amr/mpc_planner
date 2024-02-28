import datetime
import os

import control_modules

from util.code_generation import tabs, open_function, close_function, add_zero_below_10
from util.files import (
    generated_src_file,
    generated_include_file,
    solver_name,
    get_package_path,
    planner_path,
)

from util.logging import print_success, print_path


def declare_forces(header_file, cpp_file, name, solver_name):
    header_file.write(
        'extern "C"\n'
        "{\n"
        "\textern solver_int32_default "
        + solver_name
        + "_adtool2forces("
        + solver_name
        + "_float *x,  /* primal vars                                         */\n"
        "\t\t"
        + solver_name
        + "_float *y,  /* eq. constraint multiplers                           */\n"
        "\t\t"
        + solver_name
        + "_float *l,  /* ineq. constraint multipliers                        */\n"
        "\t\t"
        + solver_name
        + "_float *p,  /* parameters                                          */\n"
        "\t\t"
        + solver_name
        + "_float *f,  /* objective function (scalar)                         */\n"
        "\t\t"
        + solver_name
        + "_float *nabla_f, /* gradient of objective function                      */\n"
        "\t\t"
        + solver_name
        + "_float *c,	   /* dynamics                                            */\n"
        "\t\t"
        + solver_name
        + "_float *nabla_c, /* Jacobian of the dynamics (column major)             */\n"
        "\t\t"
        + solver_name
        + "_float *h,	   /* inequality constraints                              */\n"
        "\t\t"
        + solver_name
        + "_float *nabla_h, /* Jacobian of inequality constraints (column major)   */\n"
        "\t\t"
        + solver_name
        + "_float *hess,	   /* Hessian (column major)                              */\n"
        "\t\tsolver_int32_default stage,	   /* stage number (0 indexed)                            */\n"
        "\t\tsolver_int32_default iteration, /* iteration number of solver                          */\n"
        "\t\tsolver_int32_default threadID /* Id of caller thread 								   */);\n"
        "}\n\n"
    )
    cpp_file.write(
        'extern "C"\n{\n'
        "\t"
        + solver_name
        + "_extfunc extfunc_eval_"
        + name.lower()
        + " = &"
        + solver_name
        + "_adtool2forces;\n"
        "}\n"
    )


def declare_state(header_file, cpp_file, model):
    # The dynamic state structure
    header_file.write("\nstruct State\n" "{\n\n")

    # Declare variables for the states
    for idx, state in enumerate(model.states):
        header_file.write(f"\tdouble {state};\n")

    header_file.write("\tState(){\n" "\t\tinitialize();\n" "\t}\n\n")

    header_file.write("\n\tvoid initialize()\n" "\t{\n")

    for idx, state in enumerate(model.states):
        header_file.write("\t\t" + state + " = 0.0;\n")

    header_file.write("\t}\n")

    # open_function(header_file, cpp_file, "void print()", class_name="State")
    # cpp_file.write("\t\tRCLCPP_WARN(state_logger_, \"========== State ==========\");\n")
    # for idx, state in enumerate(model.states):
    #     if model.states_from_sensor[idx]:
    #         cpp_file.write("\t\tRCLCPP_INFO_STREAM(state_logger_, \"" + state + " \t= \" << " + state + "_);\n")
    # cpp_file.write("\t\tRCLCPP_WARN(state_logger_, \"============================\");\n")
    # close_function(cpp_file)

    header_file.write("};\n\n")


def add_reset_solver(header_file, cpp_file):
    # Reset solver function #
    open_function(header_file, cpp_file, "void reset()")
    cpp_file.write(
        "\t\tfor (long int i = 0; i < *(&_params.all_parameters + 1) - _params.all_parameters; i++)\n"
        "\t\t\t_params.all_parameters[i] = 0.0;\n\n"
        "\t\tfor (long int i = 0; i < *(&_params.xinit + 1) - _params.xinit; i++)\n"
        "\t\t\t_params.xinit[i] = 0.0;\n\n"
        "\t\tfor (size_t i = 0; i < N*nvar; i++)\n"
        "\t\t\t_params.x0[i] = 0.0;\n"
    )
    close_function(cpp_file)


def add_state_and_inputs(header_file, cpp_file, model):

    # Inputs references
    header_file.write("\n\t/* Inputs */\n")
    for i in range(0, len(model.inputs)):
        header_file.write(
            "\tdouble& "
            + model.inputs[i]
            + "(unsigned int k = 0) { return _params.x0[k * nvar + "
            + str(i)
            + "]; };\n"
        )

    header_file.write("\n\t/* States */ \n")
    for i in range(0, len(model.states)):
        header_file.write(
            "\tdouble& "
            + model.states[i]
            + "(unsigned int k) { return _params.x0[k * nvar + "
            + str(model.nu + i)
            + "]; };\n"
        )


def add_set_and_get_parameter(header_file, cpp_file):

    header_file.write(
        "\t/** @brief Set and get a solver parameter at index index of stage k */\n"
    )
    open_function(
        header_file,
        cpp_file,
        "void setParameter(int k, std::string&& parameter, double value)",
    )
    cpp_file.write(
        "\t\t_params.all_parameters[k*npar + _parameter_map[parameter].as<int>()] = value;\n"
    )
    close_function(cpp_file)

    open_function(
        header_file, cpp_file, "double getParameter(int k, std::string&& parameter)"
    )
    cpp_file.write(
        "\t\treturn _params.all_parameters[k*npar + _parameter_map[parameter].as<int>()];\n"
    )
    close_function(cpp_file)


def add_solve(header_file, cpp_file, name, forces_solver_name):
    header_file.write("\t/** @brief Solve the optimization */\n")
    open_function(header_file, cpp_file, "int solve()")
    cpp_file.write(
        "\t\tint exit_code = "
        + forces_solver_name
        + "_solve(&_params, &_output, &_info, _solver_memory_handle, stdout, extfunc_eval_"
        + name.lower()
        + ");\n"
    )
    cpp_file.write("\t\treturn exit_code;\n")
    close_function(cpp_file)


# NOTE: Do I need the "-nu" here?
def add_xinit(header_file, cpp_file, model):
    open_function(
        header_file, cpp_file, "void setXinit(std::string&& state_name, double value)"
    )
    cpp_file.write(
        "\t\t_params.xinit[_model_map[state_name][1].as<int>() - nu] = value;\n"
    )
    close_function(cpp_file)


# NOTE: first output is the initial state
def add_get_output(header_file, cpp_file, settings):
    open_function(
        header_file, cpp_file, "double getOutput(int k, std::string&& state_name)"
    )
    for k in range(settings["N"]):
        cpp_file.write(
            f"\t\tif(k == {k})\n"
            f"\t\t\treturn _output.x{k+1}[_model_map[state_name][1].as<int>()];\n"
        )
    # cpp_file.write("throw std::runtime_expc")
    close_function(cpp_file)


def generate_module_header(modules):
    path = f"{get_package_path('mpc-planner-modules')}/include/mpc-planner-modules/modules.h"
    print_path("Module Header", path, end="", tab=True)

    module_header = open(path, "w")

    module_header.write("#ifndef __MPC_PLANNER_GENERATED_MODULES_H__\n")
    module_header.write("#define __MPC_PLANNER_GENERATED_MODULES_H__\n\n")
    for module in modules.modules:
        module_header.write(f"#include <mpc-planner-modules/{module.import_name}>\n")
        for source in module.sources:
            module_header.write(f"#include <mpc-planner-modules/{source}>\n")

    module_header.write("\n")

    module_header.write("namespace MPCPlanner\n{\n")
    module_header.write("\tclass Solver;\n")
    module_header.write(
        "\tinline void initializeModules(std::vector<std::shared_ptr<ControllerModule>> &modules, std::shared_ptr<Solver> solver)\n\t{\n"
    )

    for module in modules.modules:
        module_header.write("\t\tmodules.emplace_back(nullptr);\n")
        module_header.write(
            "\t\tmodules.back() = std::make_shared<"
            + module.module_name
            + ">(solver);\n"
        )
    module_header.write("\n\t}\n")
    module_header.write("}")

    module_header.write("\n#endif")

    module_header.close()
    print_success(" -> generated")


def generate_module_definitions(modules):
    path = f"{get_package_path('mpc-planner-modules')}/include/mpc-planner-modules/definitions.h"
    print_path("Definition Header", path, end="", tab=True)

    definitions_header = open(path, "w")

    definitions_header.write("#ifndef __MPC_PLANNER_GENERATED_DEFINITIONS_H__\n")
    definitions_header.write("#define __MPC_PLANNER_GENERATED_DEFINITIONS_H__\n\n")

    for module in modules.modules:
        module.add_definitions(definitions_header)
    definitions_header.write("\n")

    definitions_header.write("\n#endif")

    definitions_header.close()
    print_success(" -> generated")


def generate_module_cmake(modules):
    path = f"{get_package_path('mpc-planner-modules')}/modules.cmake"
    print_path("Module CMake", path, end="", tab=True)
    module_cmake = open(path, "w")
    # module_cmake.write("set(MODULE_DEPENDENCIES\n")

    module_cmake.write("set(MODULE_SOURCES\n")
    for module in modules.modules:
        module_cmake.write(f"\tsrc/{module.import_name.split('.')[0]}.cpp\n")
        for source in module.sources:
            module_cmake.write(f"\tsrc/{source.split('.')[0]}.cpp\n")

    module_cmake.write(")\n")
    module_cmake.close()
    print_success(" -> generated")


def generate_cpp_code(settings, model):
    name = settings["name"].capitalize()
    forces_solver_name = solver_name(settings)

    header_file = open(generated_include_file(settings), "w")
    # cpp_file = open(generated_src_file(settings), "w")
    # cpp_file.write(f"#include \"mpc_planner_generated.h\"\n")

    header_file.write(
        "/** This file was autogenerated by the mpc-planner-solver package at "
        + datetime.datetime.now().strftime("%I:%M%p on %B %d, %Y")
        + "*/\n"
    )

    # IMPORTS
    header_file.write(
        "#ifndef __MPC_PLANNER_GENERATED_H__\n"
        "#define __MPC_PLANNER_GENERATED_H__\n\n"
        "#include <Solver.h>\n\n"
    )

    header_file.write("namespace MPCPlanner{\n\n")
    # cpp_file.write("using namespace MPCPlanner;\n\n")

    N = settings["N"]
    header_file.write(
        f"double getForcesOutput(const {forces_solver_name}_output& output, const int k, const int var_index){{\n"
    )
    for k in range(settings["N"]):
        header_file.write(f"\t\tif(k == {k})\n")
        if k == 0:
            header_file.write(
                f"\t\t\t{{\n"
                f"\t\t\t\tif(var_index >= {model.nu})"
                f'\t\t\t\t\tLOG_WARN("getForcesOutput for k = 0 returns the initial state.");\n'
            )
        header_file.write(
            f"\t\t\treturn output.x{add_zero_below_10(k+1, N)}[var_index];\n"
        )
        if k == 0:
            header_file.write("\t\t}\n")

    header_file.write(
        'throw std::runtime_error("Invalid k value for getForcesOutput");\n'
    )
    header_file.write("}\n\n")

    header_file.write(
        f"double loadForcesWarmstart({forces_solver_name}_params& params, const {forces_solver_name}_output& output){{\n"
    )
    header_file.write(f"\t\tfor (int i = 0; i < {model.nu}; i++){{\n")
    header_file.write(
        f"\t\t\tparams.z_init_{add_zero_below_10(0, N)}[i] = output.x{add_zero_below_10(1, N)}[i];\n\t\t}}\n"
    )

    header_file.write(f"\t\tfor (int i = 0; i < {model.get_nvar()}; i++){{\n")
    for k in range(1, N):
        header_file.write(
            f"\t\t\tparams.z_init_{add_zero_below_10(k, N)}[i] = output.x{add_zero_below_10(k+1, N)}[i];\n"
        )
    header_file.write("\t\t}\n\t}")

    header_file.write("}\n#endif")

    print_success(" -> generated")
    return


def generate_cpp_code_old(settings, model):
    # print("Auto generated files:")
    # write_interface_configuration(settings, model)
    # write_solver_include(settings, model)
    # write_configuration_flag(settings)
    # write_current_configuration(settings)
    # write_module_file(settings, model)
    # write_cmake_file(settings, model)
    # modify_package_xml_file(settings, model)

    name = settings["name"].capitalize()
    forces_solver_name = solver_name(settings)

    header_file = open(generated_include_file(name), "w")
    cpp_file = open(generated_src_file(name), "w")

    cpp_file.write('#include "mpc-planner-jackal/' + name + 'Solver.h"\n')

    header_file.write(
        "/** This file was autogenerated by the mpc-planner-solver package at "
        + datetime.datetime.now().strftime("%I:%M%p on %B %d, %Y")
        + "*/\n"
    )

    # IMPORTS
    header_file.write(
        "#ifndef __" + name + "MODEL_H__\n"
        "#define __" + name + "MODEL_H__\n\n"
        "#include <mpc-planner-util/load_yaml.hpp>\n"
        "#include <vector>\n"
        "#include <iostream>\n"  # Temporary!
        "#include <memory>\n\n"
        "#include <" + forces_solver_name + ".h>\n"
        "#include <" + forces_solver_name + "_memory.h>\n\n"
    )

    declare_forces(header_file, cpp_file, name, forces_solver_name)
    declare_state(header_file, cpp_file, model)

    # THE MAIN CLASS
    header_file.write("class Solver\n" "{\n\n")

    header_file.write("protected:\n")
    header_file.write("\tState _state;\n\n")
    header_file.write("\tchar *_solver_memory;\n")
    header_file.write("\t" + forces_solver_name + "_mem *_solver_memory_handle;\n\n")

    header_file.write("public:\n")
    header_file.write("\tint _solver_id;\n\n")

    header_file.write(
        "\t"
        + forces_solver_name
        + "_params _params;\n"
        + "\t"
        + forces_solver_name
        + "_output _output;\n"
        + "\t"
        + forces_solver_name
        + "_info _info;\n\n"
    )

    header_file.write(
        "\tint N;		 // Horizon length\n"
        "\tunsigned int nu;		 // Number of control variables\n"
        "\tunsigned int nx;		 // Differentiable variables\n"
        "\tunsigned int nvar; // Total variable count\n"
        "\tunsigned int npar;	 // Parameters per iteration\n"
        "\tdouble dt;\n\n"
        "\tYAML::Node _config, _parameter_map, _model_map;\n\n"
    )
    # "\tstd::string method_name_;\n"

    open_function(
        header_file,
        cpp_file,
        "Solver(int solver_id)",
        optional_header_with_defaults="Solver(int solver_id = 0)",
        has_type=False,
    )
    cpp_file.write(
        "\t_solver_id = solver_id;\n"
        "\t\t_solver_memory = (char*)malloc("
        + forces_solver_name
        + "_get_mem_size());\n"
        "\t\t_solver_memory_handle = "
        + forces_solver_name
        + "_external_mem(_solver_memory, _solver_id,"
        + forces_solver_name
        + "_get_mem_size());\n"
        '\t\tloadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "solver_settings"), _config);\n'
        '\t\tloadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "parameter_map"), _parameter_map);\n'
        '\t\tloadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "model_map"), _model_map);\n'
        '\t\tN = _config["N"].as<unsigned int>();\n'
        '\t\tnu = _config["nu"].as<unsigned int>();\n'
        '\t\tnx = _config["nx"].as<unsigned int>();\n'
        '\t\tnvar = _config["nvar"].as<unsigned int>();\n'
        '\t\tnpar = _config["npar"].as<unsigned int>();\n'
    )
    cpp_file.write('\t\tstd::cout << nx << ", " << nu << ", " << N << std::endl;\n')
    cpp_file.write("\t\treset();\n")
    close_function(cpp_file)

    add_reset_solver(header_file, cpp_file)

    open_function(header_file, cpp_file, "~Solver()")
    cpp_file.write("\t\tfree(_solver_memory);\n")
    close_function(cpp_file)

    add_state_and_inputs(
        header_file, cpp_file, model
    )  # TODO: Is it clear here that this refers to x0?

    add_set_and_get_parameter(header_file, cpp_file)

    add_xinit(header_file, cpp_file, model)

    add_solve(header_file, cpp_file, name, forces_solver_name)

    add_get_output(header_file, cpp_file, settings)

    header_file.write("};\n#endif")
    return

    header_file.write(
        "\n// Tools for copying solver data to other solvers that can run in parallel"
    )
    header_file.write("\n\tchar *GetSolverMemory() { return solver_memory_; };\n")
    open_function(
        header_file, cpp_file, "void CopyInternalSolverData(SolverInterface *other)"
    )
    cpp_file.write(
        "memcpy(solver_memory_, other->GetSolverMemory(), "
        + model.system.name
        + "FORCESNLPsolver_get_mem_size());\n"
    )
    close_function(cpp_file)

    open_function(
        header_file, cpp_file, "void CopySolverParameters(SolverInterface *other)"
    )
    cpp_file.write(
        "\t\tforces_params_ = other->forces_params_;\n"
        "\t\tinitial_plan_ = other->initial_plan_;\n"
        "\t\tplan_ = other->plan_;\n"
        "\t\tinitial_vehicle_predictions_ = other->initial_vehicle_predictions_;\n"
        "\t\toptimized_vehicle_predictions_ = other->optimized_vehicle_predictions_;\n"
    )
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void CopyInitialPlan(SolverInterface *other)")
    cpp_file.write(
        "\t\tinitial_plan_ = other->initial_plan_;\n"
        "\t\tplan_ = other->plan_;\n"
        "\t\tstate_ = other->state_;\n"
        "\t\tinitial_vehicle_predictions_ = other->initial_vehicle_predictions_;\n"
    )
    close_function(cpp_file)

    open_function(
        header_file, cpp_file, "void CopySolverOutput(SolverInterface *other)"
    )
    cpp_file.write("forces_output_ = other->forces_output_;\n")
    cpp_file.write("forces_info_ = other->forces_info_;\n")
    close_function(cpp_file)

    # Load weights of the base MPC model
    if settings.modules.contains_module(control_modules.MPCBaseModule):
        open_function(
            header_file, cpp_file, "void setWeightParameters(int k, int& param_idx)"
        )
        for idx, weight in enumerate(
            settings.weight_list
        ):  # settings.weights.weights):
            cpp_file.write(
                "\t\tsetParameter(k, param_idx, weights_." + weight + "_);\n"
            )
        close_function(cpp_file)

    # Set Parameter function #
    header_file.write(
        "\t/** @brief Set a solver parameter at index index of stage k to value */\n"
    )
    open_function(
        header_file,
        cpp_file,
        "void setParameter(unsigned int k, int& index, double value)",
    )
    cpp_file.write(
        "\t\tforces_params_.all_parameters[k*FORCES_NPAR + index] = value;\n"
    )
    cpp_file.write("\t\tindex++;\n")  # Increase the index automatically
    close_function(cpp_file)

    # Set Parameter function (rvalue)#
    header_file.write(
        "\t/** @brief Set a solver parameter at index index of stage k to value */\n"
    )
    open_function(
        header_file,
        cpp_file,
        "void setParameterSpecific(unsigned int k, int index, double value)",
    )
    cpp_file.write(
        "\t\tforces_params_.all_parameters[k*FORCES_NPAR + index] = value;\n"
    )
    close_function(cpp_file)

    header_file.write(
        "\t/** @brief Set a solver parameter at index index of stage k to value */\n"
    )
    open_function(
        header_file, cpp_file, "double getParameter(unsigned int k, unsigned int index)"
    )
    cpp_file.write("\t\treturn forces_params_.all_parameters[k*FORCES_NPAR + index];\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "double GetObjectiveValue()")
    cpp_file.write("\treturn forces_info_.pobj;\n")
    close_function(cpp_file)

    # Solve function #
    header_file.write("\t/** @brief Solve the optimization */\n")
    open_function(
        header_file,
        cpp_file,
        "int solve(int max_iterations, double tolerance)",
        optional_header_with_defaults="int solve(int max_iterations = 0, double tolerance = 1e-3)",
    )
    # open_function(header_file, cpp_file, "int solve(int max_iterations = 0, double tolerance = 1e-3);")

    # header_file.write("\tint solve(int max_iterations = 0, double tolerance = 1e-3);\n")
    # open_function(header_file, cpp_file, "int solve(int max_iterations = 0, double tolerance = 1e-3)")

    cpp_file.write(
        "\t\tif(EqualityConverged(tolerance)){ // If the eq. constraints are satisfied\n"
    )
    cpp_file.write(
        "\t\t\tlast_solution_ = forces_output_; // Save the last solution\n\n"
    )
    cpp_file.write("\t\t\tlast_safe_exit_code_ = last_exit_code_;\n")
    cpp_file.write(
        "\t\t\tlast_info_ = forces_info_; // Save the last information of the solver\n"
    )
    cpp_file.write("\t\t\thas_safe_solution_ = true;\n\t\t}\n")

    if config.use_sqp and (not config.use_scenario_constraints):
        # if settings.multi_solver:
        # raise IOError("Multi solver is not supported with SQP yet in LMPCC")
        cpp_file.write(
            "\t\tint exit_code = -1;\n"
            "\t\tfor (int solve_iteration = 0; solve_iteration < max_iterations; solve_iteration++){\n"
            "\t\t\t// We load the previous trajectory in the first iterations and reuse the previous iteration in later iterations\n"
            "\t\t\tif (solve_iteration == 0)                     // | feasible\n"
            "\t\t\t\tsetReinitialize(true); // Initialize with the shifted trajectory\n"
            "\t\t\telse if (solve_iteration >= 1)\n"
            "\t\t\t\tsetReinitialize(false);\n"
            "\t\t\t// Solve an iteration of the SQP problem and load the output\n"
            "\t\t\texit_code = "
            + model.system.name
            + "FORCESNLPsolver_solve(&forces_params_, &forces_output_, "
            "&forces_info_, solver_memory_handle_, stdout, extfunc_eval_"
            + model.system.name.lower()
            + ");\n"
            "\t\twas_safe_solution_ = EqualityConverged(tolerance);\n"
            "\t\tlast_safe_exit_code_ = exit_code;\n"
            "\t\t}\n"
            "\t\treturn exit_code;"
        )
    else:
        # cpp_file.write("\t\tint exit_code = "+model.system.name+"FORCESNLPsolver_solve(&forces_params_, &forces_output_, &forces_info_, solver_memory_handle_, stdout, extfunc_eval_"+model.system.name.lower()+");\n")
        # cpp_file.write("\t\treturn exit_code;\n")
        cpp_file.write(
            "\t\tint exit_code = "
            + model.system.name
            + "FORCESNLPsolver_solve(&forces_params_, &forces_output_, &forces_info_, solver_memory_handle_, stdout, extfunc_eval_"
            + model.system.name.lower()
            + ");\n"
        )
        cpp_file.write("\t\twas_safe_solution_ = EqualityConverged(tolerance);\n")
        cpp_file.write("\t\tlast_exit_code_ = exit_code;\n")
        cpp_file.write("\t\treturn exit_code;\n")
    close_function(cpp_file)

    header_file.write("\t/** @brief Print Solver Info for this Iteration */\n")
    open_function(header_file, cpp_file, "void printSolveInfo()")
    if config.use_sqp:
        if config.sqp_save_bfgs:
            cpp_file.write(
                '\t\tstd::cout << "Saving BFGS \\n";\n'
                "\t\tfor (size_t i = 0; i < FORCES_TOTAL_V*((FORCES_TOTAL_V- 1) / 2. + 1); i++)\n"
                '\t\t\tdata_saver_.AddData("k01_" + std::to_string(i), forces_output_.BFGSlower01[i]);\n'
                "\t\tfor (size_t i = 0; i < FORCES_TOTAL_V*((FORCES_TOTAL_V- 1) / 2. + 1); i++)\n"
                '\t\t\tdata_saver_.AddData("k09_" + std::to_string(i), forces_output_.BFGSlower09[i]);\n'
                "\t\tfor (size_t i = 0; i < FORCES_TOTAL_V*((FORCES_TOTAL_V- 1) / 2. + 1); i++)\n"
                '\t\t\tdata_saver_.AddData("k19_" + std::to_string(i), forces_output_.BFGSlower19[i]);\n'
                '\t\tdata_saver_.SaveData(RosTools::GetSharedPath("lmpcc_base") + "data/", "BFGS");\n'
            )
    elif config.multi_solver:
        cpp_file.write("\t\tstd::cout << forces_info_.it << std::endl;\n")
    close_function(cpp_file)

    # Function for safety under equality convergence
    open_function(header_file, cpp_file, "double GetPreviousEqualityTolerance()")
    cpp_file.write("\t\treturn last_info_.res_eq;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "bool EqualityConverged(double tolerance)")
    cpp_file.write("\t\treturn forces_info_.res_eq < tolerance;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "bool HasSafeSolution()")
    cpp_file.write("\t\treturn has_safe_solution_;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "bool WasSafeSolution()")
    cpp_file.write(
        "\t\treturn was_safe_solution_; // True if the last solution has converged\n"
    )
    close_function(cpp_file)

    open_function(header_file, cpp_file, "int UseLastSolution()")
    cpp_file.write("\t\tforces_output_ = last_solution_;\n")
    cpp_file.write("\t\tforces_info_ = last_info_;\n")
    cpp_file.write("\t\treturn last_safe_exit_code_;\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "double Cost() const")
    cpp_file.write("\t\treturn forces_info_.pobj;\n")
    close_function(cpp_file)

    header_file.write("\t/** @brief Reinitialize the solver (SQP) */\n")
    open_function(header_file, cpp_file, "void setReinitialize(bool value)")
    if config.use_sqp:
        cpp_file.write(
            "\t\tforces_params_.reinitialize = value;\n"
            "\t\tif(value){\n"
            "\t\t\tloadBFGS();\n"
            "\t\t\thas_safe_solution_ = false;\n"
            "\t\t}\n"
        )
    close_function(cpp_file)

    # Insert Predicted Trajectory function # TODO
    header_file.write(
        "\t/** @brief Important Note: output.x01 holds the initial state! */\n"
    )
    open_function(header_file, cpp_file, "void insertPredictedTrajectory()")
    cpp_file.write("\t\tfor (unsigned int i = 0; i < FORCES_TOTAL_V; i++){\n")

    # This needs to be N_bar in order to save all computed values
    for k in range(0, settings.N_bar):
        if k >= 9:
            cpp_file.write(
                "\t\t\tforces_params_.x0[i + "
                + str(k)
                + " * FORCES_TOTAL_V] = forces_output_.x"
                + str(k + 1)
                + "[i];\n"
            )
        else:
            cpp_file.write(
                "\t\t\tforces_params_.x0[i + "
                + str(k)
                + " * FORCES_TOTAL_V] = forces_output_.x0"
                + str(k + 1)
                + "[i];\n"
            )

    cpp_file.write("\t\t}\n")
    close_function(cpp_file)

    # Set initial spline function (value based), spline cannot be updated from the state usually #
    # Check what index the spline is
    # spline_idx = -1
    # for idx, state in enumerate(model.states):
    #     if state == "spline":
    #         spline_idx = idx

    spline_idx = model.get_state_idx("spline")
    if spline_idx != -1:
        header_file.write("\n\t/** @brief Set xinit at index to value */\n")
        open_function(header_file, cpp_file, "void setInitialSpline(double value)")
        cpp_file.write("\t\tforces_params_.xinit[" + str(spline_idx) + "] = value;\n")
        close_function(cpp_file)

    # Set initial plan to previously planned plan #
    header_file.write("\n\t// Set initial plan to previously computed plan\n")
    open_function(header_file, cpp_file, "void setInitialPlanToPreviousPlan()")
    cpp_file.write(
        "\t\tinitial_plan_ = plan_; // at initialization we set the last plan as the plan of the "
        "previous optimization\n"
    )
    close_function(cpp_file)

    header_file.write(
        "\n\t// Load predictions of the vehicle based on this initial plan\n"
    )
    open_function(header_file, cpp_file, "void LoadInitialVehiclePredictions()")
    cpp_file.write(
        "\t\tinitial_vehicle_predictions_.clear();\n"
        "\t\tfor(size_t k = 0; k < FORCES_N; k++){\n"
        "\t\t\tinitial_vehicle_predictions_.emplace_back(Eigen::Vector2d(InitialPlan(k).x(), InitialPlan(k).y()),\n"
        "\t\t\tInitialPlan(k).psi(),\n"
        "\t\t\t*area_);\n"
        "\t\t}\n"
    )
    close_function(cpp_file)

    header_file.write(
        "\n\t// Load predictions of the vehicle based on the optimized plan\n"
    )
    open_function(header_file, cpp_file, "void LoadOptimizedVehiclePredictions()")
    cpp_file.write(
        "\t\toptimized_vehicle_predictions_.clear();\n"
        "\t\tfor(size_t k = 0; k < FORCES_N; k++){\n"
        "\t\t\toptimized_vehicle_predictions_.emplace_back(Eigen::Vector2d(Plan(k).x(), Plan(k).y()),\n"
        "\t\t\tPlan(k).psi(),\n"
        "\t\t\t*area_);\n"
        "\t\t}\n"
    )
    close_function(cpp_file)

    open_function(
        header_file,
        cpp_file,
        "void LoadVehiclePredictionsToInitialPlan(const std::vector<VehicleRegion> &vehicle_prediction)",
    )
    cpp_file.write(
        "\t\t	for (size_t k = 0; k < FORCES_N; k++){\n"
        "\t\t\t		initial_plan_.states_[k].set_x(vehicle_prediction[k].pos_(0));\n"
        "\t\t\t		initial_plan_.states_[k].set_y(vehicle_prediction[k].pos_(1));\n"
        "\t\t\t		initial_plan_.states_[k].set_psi(vehicle_prediction[k].orientation_);\n"
        "\t\t\t	}\n"
    )
    close_function(cpp_file)

    # header_file.write("\t/** @brief Use the FORCES Pro Dynamics to compute the integrated trajectory based on the dynamics. */\n")
    # open_function(header_file, cpp_file, model.system.name+"DynamicsState PropagateInitialState("+model.system.name+"DynamicsState &initial_state, int n_steps)",
    #               optional_header_with_defaults=model.system.name+"DynamicsState PropagateInitialState("+model.system.name+"DynamicsState &initial_state, int n_steps=1)")
    # cpp_file.write(
    #     "\tlmpcc_solver::PropagateDynamicsRequest req;\n\n"
    #     "\tstd_msgs::msg::Float64MultiArray input_msg;\n"
    #     "\tfor (size_t k = 0; k < FORCES_N; k++){\n")
    # for input in model.inputs:
    #     cpp_file.write("\t\tinput_msg.data.push_back(" + input + "(k));\n")

    # cpp_file.write(
    #     "\t}\n"
    #     "\tstd_msgs::MultiArrayDimension dims;\n"
    #     "\tdims.size = FORCES_NU;\n"
    #     "\tinput_msg.layout.dim.push_back(dims);\n"
    #     "\tdims.size = FORCES_N;\n"
    #     "\tinput_msg.layout.dim.push_back(dims);\n"
    #     "\treq.inputs = input_msg;\n\n"
    #     "\tstd_msgs::msg::Float64MultiArray x_init_msg;\n")

    # for idx, state in enumerate(model.states):
    #     if model.states_from_sensor[idx]:
    #         cpp_file.write("\tx_init_msg.data.push_back(initial_state." + state + "());\n")
    #     else:
    #         cpp_file.write("\tx_init_msg.data.push_back(0.);\n")

    # cpp_file.write(
    #     "\treq.x_init = x_init_msg;\n"
    #     "\tdims.size = FORCES_NX;\n"
    #     "\treq.x_init.layout.dim.push_back(dims);\n\n"
    #     "\tlmpcc_solver::PropagateDynamicsResponse response;\n"
    #     "\tif (ros::service::call(\"propagate_dynamics\", req, response)){\n"
    #     "\t\t"+model.system.name+"DynamicsState output;\n")
    # for idx, state in enumerate(model.states):
    #     if model.states_from_sensor[idx]:
    #         cpp_file.write("\t\toutput.set_" + state + "(response.outputs.data[" + str(idx) + "]);\n")

    # cpp_file.write(
    #     "\t\tstate_ = output;\n"
    #     "\t\treturn output;\n"
    #         "\t}else{\n"
    #         "\t\tROS_ERROR(\"Failed to propagate the dynamics using the PropagateDynamics server (is it running?)\");\n"
    #         "\t\treturn initial_state;\n"
    #         "\t}\n")
    # close_function(cpp_file)

    # Set initial system state to xinit and x0 in solver #
    header_file.write(
        "\t/** @brief Set all initial solver values to the current state */\n"
    )
    open_function(header_file, cpp_file, "void setSolverInitialState()")
    cpp_file.write("\t\t// Fill xinit\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write(
                "\t\tforces_params_.xinit[" + str(idx) + "] = state_." + state + "();\n"
            )

    # Do not set the 0th initial guess values
    cpp_file.write("\n")
    cpp_file.write("\t\t// Fill first entries of x0\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write("\t\t" + state + "(0) = state_." + state + "();\n")

    cpp_file.write("\n")
    cpp_file.write("\t\t// Set initial plan to previously computed plan\n")
    cpp_file.write("\t\tsetInitialPlanToPreviousPlan();\n")
    close_function(cpp_file)

    open_function(
        header_file, cpp_file, model.system.name + "DynamicsState& GetInitialState()"
    )
    cpp_file.write("\t\tstatic " + model.system.name + "DynamicsState state;\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            cpp_file.write(
                "\t\tstate." + state + "() = forces_params_.xinit[" + str(idx) + "];\n"
            )
    cpp_file.write("\t\treturn state;\n")
    close_function(cpp_file)

    header_file.write("\n\t// Set solver values to sensor values\n")
    open_function(header_file, cpp_file, "void resetAtInfeasible()")
    cpp_file.write("\t\tfor (size_t k = 0; k < FORCES_NBAR; k++){\n")
    for idx, state in enumerate(model.states):
        if model.states_from_sensor[idx]:
            if model.states_from_sensor_at_infeasible[idx]:  # Set to the sensor
                cpp_file.write("\t\t\t" + state + "(k) = state_." + state + "();\n")
            else:  # Otherwise set to 0.0
                cpp_file.write("\t\t\t" + state + "(k) = 0.0;\n")
    cpp_file.write("\t\t}\n")
    close_function(cpp_file)

    header_file.write("\n\t// Load the solution of the solver into this class\n")
    open_function(
        header_file,
        cpp_file,
        "void LoadSolution(bool shift_plan_forward)",
        optional_header_with_defaults="void LoadSolution(bool shift_plan_forward=false)",
    )
    cpp_file.write(
        "\t\tinsertPredictedTrajectory(); // Loads output into x0\n\n"
        "\t\t// Save and Propagate the plan\n"
        "\t\tLoadPlan(shift_plan_forward); // Loads x0 into the plan\n"
        "\t\tLoadOptimizedVehiclePredictions(); // Load the plan into a set of vehicle predictions\n"
    )
    close_function(cpp_file)

    header_file.write(
        "\n\t// Load the computed plan into the plan object. Since the initial state is loaded in x01, we start at x02.\n"
    )
    open_function(header_file, cpp_file, "void LoadPlan(bool shifted)")
    cpp_file.write(
        "\t\tif(shifted){\n" "\t\t\tLoadShiftedPlan();\n" "\t\t\treturn;\n\t\t}\n\n"
    )

    for idx, state in enumerate(model.states):
        if not model.states_from_sensor[idx]:
            continue

        cpp_file.write("\t\t/* Load " + state + " */\n")
        for k in range(0, config.N):
            cpp_file.write(
                "\t\tplan_.states_["
                + str(k)
                + "].set_"
                + state
                + "(forces_output_.x"
                + val_zero(k + 2)
                + "["
                + str(idx + model.nu)
                + "]);\n"
            )

    # Populate the control inputs
    for name, var_name in model.control_inputs.items():
        if var_name in model.inputs:
            cpp_file.write("\tcontrol_inputs_." + name + " = " + var_name + "(0);\n")
        elif var_name in model.states:
            cpp_file.write("\tcontrol_inputs_." + name + " = " + var_name + "(1);\n")
        else:
            raise IOError(
                "Given control input variable name "
                + var_name
                + " is not part of the model"
            )

    close_function(cpp_file)

    # The shifted plan shifts the entire plan forward
    open_function(header_file, cpp_file, "void LoadShiftedPlan()")
    for idx, state in enumerate(model.states):
        if not model.states_from_sensor[idx]:
            continue

        cpp_file.write("\t\t/* Load " + state + " */\n")
        for k in range(0, config.N - 1):
            cpp_file.write(
                "\t\tplan_.states_["
                + str(k)
                + "].set_"
                + state
                + "(forces_output_.x"
                + val_zero(k + 3)
                + "["
                + str(idx + model.nu)
                + "]);\n"
            )

        # Make the terminal state the same as the panterminal state
        cpp_file.write(
            "\t\tplan_.states_["
            + str(config.N - 1)
            + "].set_"
            + state
            + "(forces_output_.x"
            + val_zero(config.N + 1)
            + "["
            + str(idx + model.nu)
            + "]);\n"
        )

    # Populate the control inputs
    for name, var_name in model.control_inputs.items():
        if var_name in model.inputs:
            cpp_file.write("\tcontrol_inputs_." + name + " = " + var_name + "(0);\n")
        elif var_name in model.states:
            cpp_file.write("\tcontrol_inputs_." + name + " = " + var_name + "(1);\n")
        else:
            raise IOError(
                "Given control input variable name "
                + var_name
                + " is not part of the model"
            )
    close_function(cpp_file)

    # Load initial plan into the correct entries of x0
    header_file.write(
        "\n\t// Load the initial guess into the solver (solver setting x0). "
        "Note that since x(0) is the initial state, we load x(1), x(2), ...\n"
    )
    open_function(header_file, cpp_file, "void loadInitialPlanAsWarmStart()")
    cpp_file.write("\t\tfor (size_t k = 0; k < FORCES_N; k++)\n\t\t{\n")

    for idx, state in enumerate(model.states):
        if not model.states_from_sensor[idx]:
            continue

        cpp_file.write(
            "\t\t\t" + state + "(k+1) = initial_plan_.states_[k]." + state + "();\n"
        )

    cpp_file.write("\t\t}\n\n")
    cpp_file.write("\t\t// Also set the final Forces state\n")
    for idx, state in enumerate(model.states):
        if not model.states_from_sensor[idx]:
            continue
        cpp_file.write("\t\t" + state + "(FORCES_N + 1) = " + state + "(FORCES_N);\n")
    close_function(cpp_file)

    # Access to the output data
    open_function(header_file, cpp_file, "double output(int k, int variable)")
    cpp_file.write("\t\tswitch(k){\n")
    for k in range(settings.N_bar):
        cpp_file.write(
            "\t\t\tcase "
            + str(k)
            + ": return forces_output_.x"
            + val_zero(k + 1)
            + "[variable];\n"
        )
    cpp_file.write(
        '\t\t\tdefault: throw std::runtime_error("Solver: incorrect output requested");\n'
        "\t\t}\n"
    )
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void setTimeout(double seconds)")
    cpp_file.write("\t\tforces_params_.solver_timeout = seconds;\n")
    close_function(cpp_file)

    # BFGS Initialization
    # @note INCORRECT BUT NOT USED
    open_function(header_file, cpp_file, "void resetBFGS()")
    if config.use_sqp and config.sqp_supply_bfgs:
        cpp_file.write("\t\tdouble value;\n")
        cpp_file.write(
            "\t\tfor (long int i = 0; i < "
            + str((int)(model.nvar * model.nvar / 2 + model.nvar / 2))
            + "; i++){\n"
        )

        idx = 0
        didx = 2
        for i in range(model.nx):
            cpp_file.write("\t\t\t")
            if i != 0:
                cpp_file.write("else ")

            cpp_file.write(
                "if(i == "
                + str(idx)
                + ") {value = "
                + str(model.bfgs_init[i, i])
                + ";}\n"
            )
            idx += didx
            didx += 1

        cpp_file.write("\t\t\telse { value = 0.0;};\n")

        for k in range(settings.N_bar):
            if k < 10:
                cpp_file.write(
                    "\t\t\tforces_params_.BFGSinitLower0" + str(k) + "[i] = value;\n"
                )
            else:
                cpp_file.write(
                    "\t\t\tforces_params_.BFGSinitLower" + str(k) + "[i] = value;\n"
                )
        cpp_file.write("\t\t}\n")
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void loadBFGS()")
    if config.use_sqp and config.sqp_supply_bfgs:
        cpp_file.write("\t\tfor(u_int i = 0; i < FORCES_TOTAL_V; i++){\n")
        for k in range(settings.N_bar):
            if k < 10:
                cpp_file.write(
                    "\t\t\tforces_params_.BFGSinitLower0"
                    + str(k)
                    + "[i] = forces_output_."
                )
            else:
                cpp_file.write(
                    "\t\t\tforces_params_.BFGSinitLower"
                    + str(k)
                    + "[i] = forces_output_."
                )

            if k + 1 < 10:
                cpp_file.write("BFGSlower0" + str(k + 1) + "[i];\n")
            else:
                cpp_file.write("BFGSlower" + str(k + 1) + "[i];\n")

        cpp_file.write("\t\t}\n")
    close_function(cpp_file)

    open_function(
        header_file,
        cpp_file,
        "void debugPrintSolverInfo(const unsigned int &w_solver_val)",
    )
    if not config.use_sqp:
        cpp_file.write(
            "\t\tunsigned int w_solver_info_table = w_solver_val + 17;\n\n"
            '\t\tstd::cout << "Printing all available solver information:" << std::endl;\n'
            "\t\tstd::cout << std::string(w_solver_info_table,'_') << std::endl;\n"
            '\t\tstd::cout << "|" << std::string(9,\' \') << "it | " << std::setw(w_solver_val)'
            ' << forces_info_.it << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(5,\' \') << "it2opt | " << std::setw(w_solver_val)'
            ' << forces_info_.it2opt << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(5,\' \') << "res_eq | " << std::setw(w_solver_val)'
            ' << forces_info_.res_eq << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(3,\' \') << "res_ineq | " << std::setw(w_solver_val)'
            ' << forces_info_.res_ineq << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(5,\' \') << "rsnorm | " << std::setw(w_solver_val)'
            ' << forces_info_.rsnorm << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(2,\' \') << "rcompnorm | " << std::setw(w_solver_val)'
            ' << forces_info_.rcompnorm << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(7,\' \') << "pobj | " << std::setw(w_solver_val)'
            ' << forces_info_.pobj << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(7,\' \') << "dobj | " << std::setw(w_solver_val)'
            ' << forces_info_.dobj << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(7,\' \') << "dgap | " << std::setw(w_solver_val)'
            ' << forces_info_.dgap << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(6,\' \') << "rdgap | " << std::setw(w_solver_val)'
            ' << forces_info_.rdgap << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(9,\' \') << "mu | " << std::setw(w_solver_val)'
            ' << forces_info_.mu << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(5,\' \') << "mu_aff | " << std::setw(w_solver_val)'
            ' << forces_info_.mu_aff << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(6,\' \') << "sigma | " << std::setw(w_solver_val)'
            ' << forces_info_.sigma << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(3,\' \') << "lsit_aff | " << std::setw(w_solver_val)'
            ' << forces_info_.lsit_aff << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(4,\' \') << "lsit_cc | " << std::setw(w_solver_val)'
            ' << forces_info_.lsit_cc << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(3,\' \') << "step_aff | " << std::setw(w_solver_val)'
            ' << forces_info_.step_aff << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(4,\' \') << "step_cc | " << std::setw(w_solver_val)'
            ' << forces_info_.step_cc << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(2,\' \') << "solvetime | " << std::setw(w_solver_val)'
            ' << forces_info_.solvetime << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(1,\' \') << "fevalstime | "'
            ' << std::setw(w_solver_val) << forces_info_.fevalstime << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(w_solver_info_table-2,\'_\') << "|" << std::endl;\n'
            "\t\tstd::cout << std::endl;\n"
            '\t\tstd::cout << "Done printing all available solver information" << std::endl;\n'
            "\t\tstd::cout << std::endl;\n"
        )
    else:
        cpp_file.write(
            "\t\tunsigned int w_solver_info_table = w_solver_val + 17;\n\n"
            '\t\tstd::cout << "Printing all available solver information:" << std::endl;\n'
            "\t\tstd::cout << std::string(w_solver_info_table,'_') << std::endl;\n"
            '\t\tstd::cout << "|" << std::string(9,\' \') << "it | " << std::setw(w_solver_val)'
            ' << forces_info_.it << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(5,\' \') << "res_eq | " << std::setw(w_solver_val)'
            ' << forces_info_.res_eq << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(5,\' \') << "rsnorm | " << std::setw(w_solver_val)'
            ' << forces_info_.rsnorm << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(7,\' \') << "pobj | " << std::setw(w_solver_val)'
            ' << forces_info_.pobj << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(2,\' \') << "solvetime | " << std::setw(w_solver_val)'
            ' << forces_info_.solvetime << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(1,\' \') << "fevalstime | "'
            ' << std::setw(w_solver_val) << forces_info_.fevalstime << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(5,\' \') << "QPtime | " << std::setw(w_solver_val)'
            ' << forces_info_.it << " |" << std::endl;\n'
            '\t\tstd::cout << "|" << std::string(w_solver_info_table-2,\'_\') << "|" << std::endl;\n'
            "\t\tstd::cout << std::endl;\n"
            '\t\tstd::cout << "Done printing all available solver information" << std::endl;\n'
            "\t\tstd::cout << std::endl;\n"
        )
    close_function(cpp_file)

    open_function(header_file, cpp_file, "void debugPrintAll()")
    cpp_file.write(
        "\t\t/* Pre-determined size and precision settings */\n"
        "\t\tconst unsigned int prec = 4;\n\n"
        "\t\tconst unsigned int w_idx = 3;\n"
        "\t\tconst unsigned int w_solver_val = prec + 7;\n\n"
        "\t\tconst unsigned int w_col1 = 13;\n"
        "\t\tconst unsigned int w_coln = w_solver_val + 2;\n\n"
        "\t\tconst unsigned int max_stage_col = 11; // w_col1=14, w_coln=12, FORCES_NBAR=15 is using full"
        " terminal width on full HD screen, just to be on the safe side\n\n"
        "\t\t/* Determine amount and sizes of tables to print and their corresponding amount of columns"
        "*/\n"
        "\t\tconst unsigned int n_tables = std::ceil(double(FORCES_NBAR)/max_stage_col);\n"
        "\t\tconst unsigned int n_stage_col_not_last = std::min(FORCES_NBAR,max_stage_col);\n"
        "\t\tconst unsigned int n_stage_col_last = FORCES_NBAR % max_stage_col;\n\n"
        "\t\t/* Start debug print statement */\n"
        "\t\tstd::cout << std::setprecision(prec);\n"
        "\t\tstd::cout << std::endl;\n"
        '\t\tstd::cout << "Printing all available information per optimization stage:" << std::endl;'
        "\n"
        '\t\tstd::cout << "IMPORTANT NOTE:" << std::endl;\n'
        '\t\tstd::cout << "k = 0 => table shows calculated inputs and initial states" << std::endl;'
        "\n"
        '\t\tstd::cout << "k = 1 => table shows calculated inputs and 1st state update" << '
        "std::endl;\n"
        '\t\tstd::cout << "..." << std::endl;\n'
        '\t\tstd::cout << "k = N => table shows calculated inputs and Nth state update" << '
        "std::endl;\n"
        '\t\tstd::cout << "The plan consists of the inputs for k in [0, N_bar-3] ([0, '
        + str(settings.N_bar - 3)
        + "]) and the states for k in [1, N_bar-2] ([1, "
        + str(settings.N_bar - 2)
        + "]), both with length N = "
        + str(config.N)
        + '" << std::endl;\n'
        '\t\tstd::cout << "The tables below display the plan for the states" << std::endl;\n\n'
        "\t\t/* Changing variables over the different tables */\n"
        "\t\t// k_start_table and n_stage_col determine together the starting stage and ending stage of"
        " the table (thereby implicitly defining the table size)\n"
        "\t\tunsigned int k_start_table = 0;\n"
        "\t\tunsigned int n_stage_col;\n"
        "\t\tunsigned int w_table;\n"
        "\t\tstd::string array_name;\n"
        "\t\tbool is_last_table;\n\n"
        "\t\t/* Print all stage tables */\n"
        "\t\tfor (unsigned int table_idx = 0; table_idx < n_tables; table_idx++) {\n"
        "\t\t\t/* Determine table size (especially important in case of last table, which might deviate"
        " from the rest) */\n"
        "\t\t\tif (table_idx == n_tables-1 && n_stage_col_last != 0)\n"
        "\t\t\t\tn_stage_col = n_stage_col_last;\n"
        "\t\t\telse\n"
        "\t\t\t\tn_stage_col = n_stage_col_not_last;\n"
        "\t\t\tw_table = w_col1 + 1 + n_stage_col*(w_coln + 1) + 1;\n\n"
        "\t\t\tif (table_idx == n_tables - 1)\n"
        "\t\t\t\tis_last_table = true;\n\n"
        "\t\t\t/* Call printing functionality for each table */\n"
        "\t\t\tdebugPrintTableBegin(w_table, k_start_table, n_stage_col, w_col1, w_coln, is_last_table);"
        "\n\n"
        "\t\t\t// Print all solver settings\n"
        '\t\t\tdebugPrintTableVariablesString(w_table, "Solver settings");\n'
        "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, true);\n"
        "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
        "\t\t\tif (table_idx == 0) {\n"
        "\t\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
        ' w_solver_val, "xinit"); // xinit (only in first table)\n'
        "\t\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
        "\t\t\t}\n"
        "\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
        ' w_solver_val, "x0"); // x0\n'
        "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
        "\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
        ' w_solver_val, "params"); // params\n'
        "\t\t\tdebugPrintTableEmptyLine(w_table);\n\n"
        "\t\t\t// Print all solver outputs\n"
        '\t\t\tdebugPrintTableVariablesString(w_table, "Solver outputs");\n'
        "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, true);\n"
        "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
        "\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
        ' w_solver_val, "inputs"); // inputs\n'
        "\t\t\tdebugPrintTableEmptyColLine(n_stage_col, w_col1, w_coln, false);\n"
        "\t\t\tdebugPrintTableArrayEntry(k_start_table, n_stage_col, w_col1, w_coln, w_idx,"
        ' w_solver_val, "states"); // states\n'
        "\t\t\tdebugPrintTableEnd(w_table);\n\n\n"
        "\t\t\t/* Update starting stage index for next table */\n"
        "\t\t\tk_start_table += n_stage_col;\n"
        "\t\t}\n\n"
        "\t\t/* End debug print statement */\n"
        "\t\tstd::cout << std::endl;\n"
        '\t\tstd::cout << "Done printing all available information per optimization stage" <<'
        " std::endl;\n"
        "\t\tstd::cout << std::endl;\n\n"
        "\t\t/* Print all solver information */\n"
        "\t\tdebugPrintSolverInfo(w_solver_val);\n"
    )
    close_function(cpp_file)

    open_function(
        header_file,
        cpp_file,
        "void debugPrintTableBegin(const unsigned int &w_table,"
        " const unsigned int &k_start_table, const unsigned int &n_stage_col,"
        " const unsigned int &w_col1, const unsigned int &w_coln, bool &is_last_table)",
    )
    cpp_file.write(
        "\t\tstd::cout << std::string(w_table,'_') << std::endl;\n"
        "\t\tstd::cout << \"|\" << std::string(w_col1,' ');\n"
        "\t\tfor (unsigned int k = k_start_table; k < k_start_table+n_stage_col; k++) {\n"
        "\t\t\tstd::cout << \"|\" << std::string(w_coln-8,' ');\n"
        "\t\t\tif (k < 10)\n"
        '\t\t\t\tstd::cout << "  ";\n'
        "\t\t\telse if (k < 100)\n"
        '\t\t\t\tstd::cout << " ";\n'
        '\t\t\tstd::cout << "k = " << k << " ";\n'
        "\t\t}\n"
        '\t\tstd::cout << "|" << std::endl;\n'
        '\t\tstd::cout << "|" << std::string(w_table-2,\'=\') << "|" << std::endl;\n\n'
        "\t\t// Indicate start and end of plan\n"
        "\t\tif (k_start_table == 0) {\n"
        '\t\t\tstd::cout << "|" << std::string(w_col1+1+w_coln,\'=\') << "| Start of Plan ";\n'
        "\t\t\tif (!is_last_table) {\n"
        "\t\t\t\tstd::cout << std::string(w_table-1-w_col1-1-w_coln-16-2,'.') << \" |\" << std::endl;\n"
        "\t\t\t} else {\n"
        "\t\t\t\tstd::cout << std::string(w_table-1-w_col1-1-w_coln-16-14-w_coln-1,'.')"
        ' << " end of Plan |" << std::string(w_coln,\'=\') << "|" << std::endl;\n'
        "\t\t\t\treturn;\n"
        "\t\t\t}\n"
        "\t\t} else if (!is_last_table) {\n"
        '\t\t\tstd::cout << "| " << std::string(w_table-4,\'.\') << " |" << std::endl;\n'
        "\t\t}\n"
        "\t\tif (is_last_table) {\n"
        '\t\t\tstd::cout << "| " << std::string(w_table-2-14-w_coln-1,\'.\') << " end of Plan |"'
        " << std::string(w_coln,'=') << \"|\" << std::endl;\n"
        "\t\t}\n"
    )
    close_function(cpp_file)

    open_function(
        header_file, cpp_file, "void debugPrintTableEnd(const unsigned int &w_table)"
    )
    cpp_file.write(
        '\t\tstd::cout << "|" << std::string(w_table-2,\'_\') << "|" << std::endl;\n'
    )
    close_function(cpp_file)

    open_function(
        header_file,
        cpp_file,
        "void debugPrintTableEmptyLine(const unsigned int &w_table)",
    )
    cpp_file.write(
        '\t\tstd::cout << "|" << std::string(w_table-2,\' \') << "|" << std::endl;\n'
    )
    close_function(cpp_file)

    open_function(
        header_file,
        cpp_file,
        "void debugPrintTableEmptyColLine(const unsigned int &n_stage_col,"
        " const unsigned int &w_col1, const unsigned int &w_coln, const bool &is_idx_line)",
    )
    cpp_file.write(
        "\t\tfor (unsigned int k = 0; k < n_stage_col+1; k++) {\n"
        "\t\t\tif (k == 0) {\n"
        "\t\t\t\tif (is_idx_line)\n"
        '\t\t\t\t\tstd::cout << "|" << std::string(w_col1-4,\' \') << "idx ";\n'
        "\t\t\t\telse\n"
        "\t\t\t\t\tstd::cout << \"|\" << std::string(w_col1,' ');\n"
        "\t\t\t}\n"
        "\t\t\telse if (k < n_stage_col)\n"
        "\t\t\t\tstd::cout << \"|\" << std::string(w_coln,' ');\n"
        "\t\t\telse\n"
        '\t\t\t\tstd::cout << "|" << std::string(w_coln,\' \') << "|" << std::endl;\n'
        "\t\t}\n"
    )
    close_function(cpp_file)

    open_function(
        header_file,
        cpp_file,
        "void debugPrintTableVariablesString(const unsigned int &w_table, "
        "const std::string &name)",
    )
    cpp_file.write(
        '\t\tstd::cout << "|" << std::string(w_table-2,\'-\') << "|" << std::endl;\n'
        "\t\tunsigned int str_len = name.length();\n"
        '\t\tstd::cout << "| " << name << std::string(w_table-str_len-3,\' \') << "|" << std::endl;\n'
        '\t\tstd::cout << "|" << std::string(w_table-2,\'-\') << "|" << std::endl;\n'
    )
    close_function(cpp_file)

    open_function(
        header_file,
        cpp_file,
        "void debugPrintTableArrayEntry(const unsigned int &k_start_table,"
        " const unsigned int &n_stage_col, const unsigned int &w_col1,"
        " const unsigned int &w_coln, const unsigned int &w_idx,"
        "const unsigned int &w_solver_val, const std::string &name)",
    )
    cpp_file.write(
        "\t\tunsigned int idx_max = 0;\n\n"
        "\t\tbool is_xinit = false;\n"
        "\t\tbool is_x0 = false;\n"
        "\t\tbool is_params = false;\n"
        "\t\tbool is_inputs = false;\n"
        "\t\tbool is_states = false;\n\n"
        '\t\tif (!name.compare("xinit")) {\n'
        "\t\t\tis_xinit = true;\n"
        "\t\t\tidx_max = FORCES_NX;\n"
        '\t\t} else if (!name.compare("x0")) {\n'
        "\t\t\tis_x0 = true;\n"
        "\t\t\tidx_max = FORCES_TOTAL_V;\n"
        '\t\t} else if (!name.compare("params")) {\n'
        "\t\t\tis_params = true;\n"
        "\t\t\tidx_max = FORCES_NPAR;\n"
        '\t\t} else if (!name.compare("inputs")) {\n'
        "\t\t\tis_inputs = true;\n"
        "\t\t\tidx_max = FORCES_NU;\n"
        '\t\t} else if (!name.compare("states")) {\n'
        "\t\t\tis_states = true;\n"
        "\t\t\tidx_max = FORCES_NX;\n"
        "\t\t}\n\n"
        "\t\tfor (unsigned int idx = 0; idx < idx_max; idx++) {\n"
        "\t\t\tif (idx == 0) {\n"
        "\t\t\t\tunsigned int str_len = name.length();\n"
        '\t\t\t\tstd::cout << "|" << std::string(w_col1-str_len-w_idx-3,\' \') << name << ": " <<'
        ' std::setw(w_idx) << idx << " ";\n'
        "\t\t\t} else\n"
        "\t\t\t\tstd::cout << \"|\" << std::string(w_col1-w_idx-1,' ') << std::setw(w_idx) << idx <<"
        ' " ";\n'
        "\t\t\tfor (unsigned int k = k_start_table; k < k_start_table+n_stage_col; k++) {\n"
        "\t\t\t\tif (is_xinit) {\n"
        "\t\t\t\t\tif (k == 0) // xinit only exists in first stage (0)\n"
        '\t\t\t\t\t\tstd::cout << "| " << std::setw(w_solver_val) << forces_params_.xinit[idx] <<'
        ' " ";\n'
        "\t\t\t\t\telse\n"
        "\t\t\t\t\t\tstd::cout << \"|\" << std::string(w_coln,' ');\n"
        "\t\t\t\t} else if (is_x0) {\n"
        '\t\t\t\t\tstd::cout << "| " << std::setw(w_solver_val) <<'
        ' forces_params_.x0[k*FORCES_TOTAL_V + idx] << " ";\n'
        "\t\t\t\t} else if (is_params) {\n"
        '\t\t\t\t\tstd::cout << "| " << std::setw(w_solver_val) <<'
        ' forces_params_.all_parameters[k*FORCES_NPAR + idx] << " ";\n'
        "\t\t\t\t} else if (is_inputs) {\n"
        '\t\t\t\t\tstd::cout << "| " << std::setw(w_solver_val) << output(k,idx) << " ";\n'
        "\t\t\t\t} else if (is_states) {\n"
        '\t\t\t\t\tstd::cout << "| " << std::setw(w_solver_val) << output(k,FORCES_NU + idx)'
        ' << " ";\n'
        "\t\t\t\t}\n"
        "\t\t\t}\n"
        '\t\t\tstd::cout << "|" << std::endl;\n'
        "\t\t}\n"
    )
    close_function(cpp_file)

    # Add set functions for all parameters defined by name
    # for key, param in settings.params.parameters.items():
    #     open_function(header_file, cpp_file, "void set_" + param + "(unsigned int k, double value)")
    #     idx = getattr(settings.params, param + "_index")
    #     cpp_file.write("\t\tsetParameterSpecific(k, " + str(idx) + ", value);\n")
    #     close_function(cpp_file)

    header_file.write("};\n")
    header_file.write("#endif\n")
    header_file.close()
