import os
import datetime

from util.code_generation import tabs, open_function, close_function, add_zero_below_10
from util.files import generated_src_file, generated_include_file, solver_name, get_package_path, planner_path, get_current_package
from util.files import generated_parameter_include_file

from util.logging import print_success, print_path


def generate_module_header(modules):
    path = f"{get_package_path('mpc_planner_modules')}/include/mpc_planner_modules/modules.h"
    print_path("Module Header", path, end="", tab=True)

    module_header = open(path, "w")

    module_header.write("#ifndef __MPC_PLANNER_GENERATED_MODULES_H__\n")
    module_header.write("#define __MPC_PLANNER_GENERATED_MODULES_H__\n\n")
    for module in modules.modules:
        module_header.write(f"#include <mpc_planner_modules/{module.import_name}>\n")
        for source in module.sources:
            module_header.write(f"#include <mpc_planner_modules/{source}>\n")

    module_header.write("\n")

    module_header.write("namespace MPCPlanner\n{\n")
    module_header.write("\tclass Solver;\n")
    module_header.write(
        "\tinline void initializeModules(std::vector<std::shared_ptr<ControllerModule>> &modules, std::shared_ptr<Solver> solver)\n\t{\n"
    )

    for module in modules.modules:
        module_header.write("\t\tmodules.emplace_back(nullptr);\n")
        module_header.write("\t\tmodules.back() = std::make_shared<" + module.module_name + ">(solver);\n")
    module_header.write("\n\t}\n")
    module_header.write("}")

    module_header.write("\n#endif")

    module_header.close()
    print_success(" -> generated")


def generate_module_definitions(modules):
    path = f"{get_package_path('mpc_planner_modules')}/include/mpc_planner_modules/definitions.h"
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
    path = f"{get_package_path('mpc_planner_modules')}/modules.cmake"
    print_path("Module CMake", path, end="", tab=True)
    module_cmake = open(path, "w")

    module_cmake.write("if(USE_ROS2)\n")  # ROS2 find package
    dependencies = []
    for module in modules.modules:
        for dependency in module.dependencies:
            if dependency not in dependencies:
                dependencies.append(dependency)
                module_cmake.write(f"\tfind_package({dependency} REQUIRED)\n")
    module_cmake.write("endif()\n")

    module_cmake.write("set(MODULE_DEPENDENCIES\n")  # Dependencies
    dependencies = []
    for module in modules.modules:
        for dependency in module.dependencies:
            if dependency not in dependencies:
                dependencies.append(dependency)
                module_cmake.write(f"\t{dependency}\n")
    module_cmake.write(")\n\n")

    module_cmake.write("set(MODULE_SOURCES\n")  # Sources
    for module in modules.modules:
        module_cmake.write(f"\tsrc/{module.import_name.split('.')[0]}.cpp\n")
        for source in module.sources:
            module_cmake.write(f"\tsrc/{source.split('.')[0]}.cpp\n")

    module_cmake.write(")\n")
    module_cmake.close()
    print_success(" -> generated")


def generate_module_packagexml(modules):
    path = f"{get_package_path('mpc_planner_modules')}/package.xml"
    print_path("Package.xml", path, end="", tab=True)

    line_start = "<!-- START SOLVER DEPENDENT -->"
    line_end = "<!-- END SOLVER DEPENDENT -->"

    # Create a list to store lines to append
    new_lines = []
    found_start = False
    found_end = False

    # Step 1: Open the file in read mode
    with open(path, "r") as file:
        for line in file:
            # Step 3: Check if the current line matches the target line
            if line_start in line:
                new_lines.append(line)
                found_start = True  # Set the flag to True
                for module in modules.modules:
                    for dep in module.dependencies:
                        new_lines.append("\t<depend>" + dep + "</depend>\n")

            if line_end in line:
                found_end = True

            if found_start and not found_end:
                continue

            # If the target line hasn't been found, add it to the list
            new_lines.append(line)

    # If nothing was out of place with the tags
    if found_start and found_end:
        with open(path, "w") as file:
            file.writelines(new_lines)

    print_success(" -> modified")


def generate_cpp_code(settings, model):
    forces_solver_name = solver_name(settings)
    header_file_name, cpp_file_name = generated_include_file(settings)

    header_file = open(header_file_name, "w")
    cpp_file = open(cpp_file_name, "w")

    header_file.write(
        "/** This file was autogenerated by the mpc_planner_solver package at "
        + datetime.datetime.now().strftime("%I:%M%p on %B %d, %Y")
        + "*/\n"
    )

    # IMPORTS
    header_file.write("#ifndef __MPC_PLANNER_GENERATED_H__\n" "#define __MPC_PLANNER_GENERATED_H__\n\n" "#include <Solver.h>\n\n")

    header_file.write("namespace MPCPlanner{\n\n")

    cpp_file.write("#include <mpc_planner_generated.h>\n\n")
    cpp_file.write("#include <ros_tools/logging.h>\n\n")
    cpp_file.write("#include <stdexcept>\n\n")
    cpp_file.write("namespace MPCPlanner{\n\n")

    N = settings["N"]
    header_file.write(f"double getForcesOutput(const {forces_solver_name}_output& output, const int k, const int var_index);\n")
    cpp_file.write(f"double getForcesOutput(const {forces_solver_name}_output& output, const int k, const int var_index){{\n")
    for k in range(settings["N"]):
        cpp_file.write(f"\t\tif(k == {k})\n")
        if k == 0:
            cpp_file.write(
                f"\t\t\t{{\n"
                f"\t\t\t\tif(var_index >= {model.nu})"
                f'\t\t\t\t\tLOG_WARN("getForcesOutput for k = 0 returns the initial state.");\n'
            )
        cpp_file.write(f"\t\t\treturn output.x{add_zero_below_10(k+1, N)}[var_index];\n")
        if k == 0:
            cpp_file.write("\t\t}\n")

    cpp_file.write('throw std::runtime_error("Invalid k value for getForcesOutput");\n')
    cpp_file.write("}\n\n")

    header_file.write(f"void loadForcesWarmstart({forces_solver_name}_params& params, const {forces_solver_name}_output& output);\n")
    cpp_file.write(f"void loadForcesWarmstart({forces_solver_name}_params& params, const {forces_solver_name}_output& output){{\n")
    if not settings["solver_settings"]["forces"]["use_sqp"]:
        cpp_file.write(f"\t\tfor (int i = 0; i < {model.nu}; i++){{\n")
        cpp_file.write(f"\t\t\tparams.z_init_{add_zero_below_10(0, N)}[i] = params.x0[i];\n\t\t}}\n")

        cpp_file.write(f"\t\tfor (int i = 0; i < {model.get_nvar()}; i++){{\n")
        for k in range(1, N):
            cpp_file.write(f"\t\t\tparams.z_init_{add_zero_below_10(k, N)}[i] = params.x0[{model.get_nvar()}*{k} + i];\n")
        cpp_file.write("\t\t}\n")
    cpp_file.write("\t}\n")

    header_file.write("\tvoid setForcesReinitialize(Solver_params& params, const bool value);\n")
    cpp_file.write("\tvoid setForcesReinitialize(Solver_params& params, const bool value){\n")
    if settings["solver_settings"]["forces"]["use_sqp"]:
        cpp_file.write(f"\t\tparams.reinitialize = value;\n")
    cpp_file.write("\t}\n")

    header_file.write("}\n#endif")
    cpp_file.write("}\n")

    print_success(" -> generated")
    return


def generate_parameter_cpp_code(settings, model):
    header_file_name, cpp_file_name = generated_parameter_include_file(settings)

    header_file = open(header_file_name, "w")
    cpp_file = open(cpp_file_name, "w")

    header_file.write(
        "/** This file was autogenerated by the mpc_planner_solver package at "
        + datetime.datetime.now().strftime("%I:%M%p on %B %d, %Y")
        + "*/\n"
    )

    # IMPORTS
    header_file.write("#ifndef __MPC_PLANNER_PARAMETERS_H__\n")
    header_file.write("#define __MPC_PLANNER_PARAMETERS_H__\n\n")

    if settings["solver_settings"]["solver"] == "acados":  # Forward declare
        header_file.write("namespace MPCPlanner{\n\n")
        header_file.write("struct AcadosParameters;\n")
    cpp_file.write("#include <mpc_planner_parameters.h>\n\n")

    if settings["solver_settings"]["solver"] == "acados":
        cpp_file.write("#include <mpc_planner_solver/solver_interface.h>\n")
        param_name = "AcadosParameters"
    else:
        header_file.write("#include <Solver.h>\n\n")
        header_file.write("namespace MPCPlanner{\n\n")
        param_name = "Solver_params"

    cpp_file.write("namespace MPCPlanner{\n\n")

    for key, indices in settings["params"].parameter_bundles.items():
        function_name = key.replace("_", " ").title().replace(" ", "")

        if len(indices) == 1:
            header_file.write(f"void setSolverParameter{function_name}(int k, {param_name}& params, const double value, int index=0);\n")
            cpp_file.write(f"void setSolverParameter{function_name}(int k, {param_name}& params, const double value, int index){{\n")
            cpp_file.write("\t(void)index;\n")
            cpp_file.write(f"\tparams.all_parameters[k * {settings['params'].length()} + {indices[0]}] = value;\n")
        else:
            header_file.write(f"void setSolverParameter{function_name}(int k, {param_name}& params, const double value, int index);\n")
            cpp_file.write(f"void setSolverParameter{function_name}(int k, {param_name}& params, const double value, int index){{\n")
            for i, index in enumerate(indices):
                if i == 0:
                    cpp_file.write(f"\tif(index == {i})\n")
                else:
                    cpp_file.write(f"\telse if(index == {i})\n")

                cpp_file.write(f"\t\tparams.all_parameters[k * {settings['params'].length()} + {index}] = value;\n")

        cpp_file.write("}\n")

    header_file.write("}\n#endif")
    cpp_file.write("}\n")

    print_success(" -> generated")
    return


def generate_rqtreconfigure(settings):
    current_package = get_current_package()
    path = f"{get_package_path(current_package)}/cfg/"
    os.makedirs(path, exist_ok=True)
    path += "settings.cfg"
    print_path("RQT Reconfigure", path, end="", tab=True)
    rqt_file = open(path, "w")

    rqt_file.write("#!/usr/bin/env python\n")
    rqt_file.write(f'PACKAGE = "{current_package}"\n')
    rqt_file.write("from dynamic_reconfigure.parameter_generator_catkin import *\n")
    rqt_file.write("gen = ParameterGenerator()\n\n")

    rqt_file.write('weight_params = gen.add_group("Weights", "Weights")\n')
    rqt_params = settings["params"].rqt_params
    for idx, param in enumerate(rqt_params):
        rqt_file.write(
            f'weight_params.add("{param}", double_t, 1, "{param}", 1.0, {settings["params"].rqt_param_min_values[idx]}, {settings["params"].rqt_param_max_values[idx]})\n'
        )
    rqt_file.write(f'exit(gen.generate(PACKAGE, "{current_package}", ""))\n')
    rqt_file.close()
    print_success(" -> generated")

    current_package = get_current_package()
    path = f"{get_package_path(current_package)}/include/{current_package}/"
    os.makedirs(path, exist_ok=True)
    system_name = "".join(current_package.split("_")[2:])
    path += f"{system_name}_reconfigure.h"
    print_path("Reconfigure Header", path, end="", tab=True)
    rqt_header = open(path, "w")

    class_name = f"{system_name.capitalize()}Reconfigure"
    rqt_header.write("#ifndef __GENERATED_RECONFIGURE_H\n")
    rqt_header.write("#define __GENERATED_RECONFIGURE_H\n\n")
    rqt_header.write("#include <ros/ros.h>\n\n")
    rqt_header.write("#include <ros_tools/logging.h>\n")
    rqt_header.write("#include <mpc_planner_util/parameters.h>\n\n")
    rqt_header.write("// Dynamic Reconfigure server\n")
    rqt_header.write("#include <dynamic_reconfigure/server.h>\n")
    rqt_header.write(f"#include <{current_package}/Config.h>\n\n")
    rqt_header.write(f"class {class_name}\n")
    rqt_header.write("{\n")
    rqt_header.write("public:\n")
    rqt_header.write(f"\t{class_name}()\n")
    rqt_header.write("\t{\n")
    rqt_header.write("\t\t// Initialize the dynamic reconfiguration\n")
    rqt_header.write('\t\tLOG_INFO("Setting up dynamic_reconfigure server for the parameters");\n')
    rqt_header.write("\t\t// first_reconfigure_callback_ = true;\n")
    rqt_header.write(f'\t\tros::NodeHandle nh_reconfigure("{current_package}");\n')
    rqt_header.write(
        f"\t\t_reconfigure_server.reset(new dynamic_reconfigure::Server<{current_package}::Config>(_reconfig_mutex, nh_reconfigure));\n"
    )
    rqt_header.write(f"\t\t_reconfigure_server->setCallback(boost::bind(&{class_name}::reconfigureCallback, this, _1, _2));\n")
    rqt_header.write("\t}\n")
    rqt_header.write(f"\tvoid reconfigureCallback({current_package}::Config &config, uint32_t level)\n")
    rqt_header.write("\t{\n")
    rqt_header.write("\t\t(void)level;\n")
    rqt_header.write("\t\tif (_first_reconfigure_callback){\n")
    for idx, param in enumerate(rqt_params):
        rqt_header.write(f'\t\t\tconfig.{param} = CONFIG{settings["params"].rqt_param_config_names[idx](param)}.as<double>();\n')
    rqt_header.write("\t\t\t_first_reconfigure_callback = false;\n")
    rqt_header.write("\t\t}else{\n")
    for idx, param in enumerate(rqt_params):
        rqt_header.write(f'\t\t\tCONFIG{settings["params"].rqt_param_config_names[idx](param)} = config.{param};\n')
    rqt_header.write("\t\t}\n")

    rqt_header.write("\t}\n\n")
    rqt_header.write("private:\n")
    rqt_header.write("\tbool _first_reconfigure_callback{true};\n")
    rqt_header.write("\t// RQT Reconfigure ROS1\n")
    rqt_header.write(f"\tboost::shared_ptr<dynamic_reconfigure::Server<{current_package}::Config>> _reconfigure_server;\n")
    rqt_header.write("\tboost::recursive_mutex _reconfig_mutex;\n")
    rqt_header.write("};\n\n")
    rqt_header.write("#endif // __GENERATED_RECONFIGURE_H\n")
    rqt_header.close()
    print_success(" -> generated")


def generate_ros2_rqtreconfigure(settings):
    current_package = get_current_package()
    path = f"{get_package_path(current_package)}/include/{current_package}/"
    os.makedirs(path, exist_ok=True)
    system_name = "".join(current_package.split("_")[2:])
    path += f"{system_name}_ros2_reconfigure.h"
    print_path("ROS2 Reconfigure Header", path, end="", tab=True)
    rqt_header = open(path, "w")
    rqt_params = settings["params"].rqt_params
    class_name = f"{system_name.capitalize()}Reconfigure"

    rqt_header.write("#ifndef __GENERATED_ROS2_RECONFIGURE_H\n")
    rqt_header.write("#define __GENERATED_ROS2_RECONFIGURE_H\n")
    rqt_header.write("#include <rclcpp/rclcpp.hpp>\n")
    rqt_header.write("#include <ros_tools/logging.h>\n")
    rqt_header.write("#include <ros_tools/ros2_wrappers.h>\n")
    rqt_header.write("#include <mpc_planner_util/parameters.h>\n")
    rqt_header.write("template <class T>\n")
    rqt_header.write("bool updateParam(const std::vector<rclcpp::Parameter> &params, const std::string &name, YAML::Node value)\n")
    rqt_header.write("{\n")
    rqt_header.write("\tconst auto itr = std::find_if(\n")
    rqt_header.write("\t\tparams.cbegin(), params.cend(),\n")
    rqt_header.write("\t\t[&name](const rclcpp::Parameter &p)\n")
    rqt_header.write("\t\t{ return p.get_name() == name; });\n")
    rqt_header.write("\n")
    rqt_header.write("\t// Not found\n")
    rqt_header.write("\tif (itr == params.cend())\n")
    rqt_header.write("\t{\n")
    rqt_header.write("\t\treturn false;\n")
    rqt_header.write("\t}\n")
    rqt_header.write("\n")
    rqt_header.write("\tvalue = itr->template get_value<T>();\n")
    rqt_header.write('\tLOG_INFO("Parameter " + name + " set to " + std::to_string(value.as<T>()));\n')
    rqt_header.write("\treturn true;\n")
    rqt_header.write("}\n")
    rqt_header.write("\n")
    rqt_header.write(f"class {class_name}\n")
    rqt_header.write("{\n")
    rqt_header.write("public:\n")
    rqt_header.write(f"\t{class_name}(rclcpp::Node *node)\n")
    rqt_header.write("\t{\n")
    rqt_header.write('\t\tLOG_INFO("Setting up dynamic_reconfigure parameters");\n')
    rqt_header.write("\n")
    rqt_header.write("\t\tdeclareROSParameters(node);\n")
    rqt_header.write("\n")
    rqt_header.write("\t\t_set_param_res = node->add_on_set_parameters_callback(\n")
    rqt_header.write(f"\t\t\tstd::bind(&{class_name}::updateROSParameters, this, std::placeholders::_1));\n")
    rqt_header.write("\t}\n")
    rqt_header.write("\n")
    rqt_header.write("\tvirtual void declareROSParameters(rclcpp::Node *node)\n")
    rqt_header.write("\t{\n")

    for idx, param in enumerate(rqt_params):
        rqt_header.write(
            f"\t\tnode->declare_parameter<double>(\"{param}\", CONFIG{settings['params'].rqt_param_config_names[idx](param)}.as<double>());\n"
        )

    rqt_header.write("\t}\n")
    rqt_header.write("\n")
    rqt_header.write(
        "\tvirtual rcl_interfaces::msg::SetParametersResult updateROSParameters(const std::vector<rclcpp::Parameter> &parameters)\n"
    )
    rqt_header.write("\t{\n")
    for idx, param in enumerate(rqt_params):
        rqt_header.write(
            f"\t\tupdateParam<double>(parameters, \"{param}\", CONFIG{settings['params'].rqt_param_config_names[idx](param)});\n"
        )

    rqt_header.write("\n")
    rqt_header.write("\t\tauto result = rcl_interfaces::msg::SetParametersResult();\n")
    rqt_header.write("\t\tresult.successful = true;\n")
    rqt_header.write("\n")
    rqt_header.write("\n")
    rqt_header.write("\t\treturn result;\n")
    rqt_header.write("\t}\n")
    rqt_header.write("\n")
    rqt_header.write("protected:\n")
    rqt_header.write("\trclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _set_param_res;\n")
    rqt_header.write("};\n")
    rqt_header.write("#endif // __GENERATED_ROS2_RECONFIGURE_H\n")
    rqt_header.close()
    print_success(" -> generated")


# TODO Print BFGS when enabled
# std::cout << "1: options.bfgs_init = np.diag(np.array([";
# 	for (int i = 0; i < 8; i++)
# 	{
# 		std::cout << output.BFGSdiagonal01[i];
# 		if (i != 7)
# 			std::cout << ", ";
# 	}
# 	std::cout << "]))" << std::endl;
