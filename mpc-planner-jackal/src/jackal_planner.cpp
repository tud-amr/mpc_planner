#include <iostream>

#include <mpc-planner/planner.h>
#include <mpc-planner-solver/solver_interface.h>

#include <mpc-planner-types/realtime_data.h>

#include <mpc-planner-util/load_yaml.hpp>
#include <mpc-planner-util/logging.h>

using namespace MPCPlanner;

// Load ROS2 parameters
void loadROS2Parameters()
{
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    LOG_INFO("Started Jackal Planner");

    // Load parameters from file
    YAML::Node config;
    loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "settings"), config);

    Planner planner(config);

    RealTimeData data;
    State state;
    state.set("x", 10.);
    state.set("y", 0.);
    state.set("v", 1.);

    planner.SolveMPC(state, data);

    // loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "solver_settings"), config);
    // std::cout << config["nu"].as<int>() << std::endl;

    /*
    // These are tests
    Solver solver;

    solver.setParameter(0, "acceleration", 3.0);
    solver.setParameter(0, "angular_velocity", 4.0);
    assert(solver.getParameter(0, "angular_velocity") == 4.0);
    assert(solver.getParameter(0, "acceleration") == 3.0);

    solver.setXinit("x", 10.0);
    solver.setXinit("y", 20.0);
    assert(solver._params.xinit[0] == 10.0);
    assert(solver._params.xinit[1] == 20.0);

    // Test run for the solver
    solver.setXinit("x", 0.0);
    solver.setXinit("y", 0.0);
    solver.setXinit("v", 1.0);

    for (int k = 0; k < solver.N; k++)
    {
        solver.setParameter(k, "acceleration", 1.0);
        solver.setParameter(k, "angular_velocity", 3.0);
        assert(solver.getParameter(k, "acceleration") == 1.0);
        assert(solver.getParameter(k, "angular_velocity") == 3.0);
    }

    int exitflag = solver.solve();
    std::cout << "Exit flag: " << exitflag << std::endl;
    assert(exitflag == 1);

    // Read the solution
    std::cout << solver.getOutput(0, "a") << std::endl;
    std::cout << solver.getOutput(0, "w") << std::endl;
    std::cout << solver.getOutput(2, "x") << std::endl;
    std::cout << solver.getOutput(2, "y") << std::endl;
    */
    return 0;
}
