#include <mpc-planner/planner.h>

#include <mpc-planner-types/controller_module.h>
#include <mpc-planner-types/realtime_data.h>
#include <mpc-planner-solver/solver_interface.h>

/** @todo Generate */
#include <mpc-planner/mpc_base.h>
#include <mpc-planner/goal_module.h>

#include <mpc-planner-util/load_yaml.hpp>
#include <mpc-planner-util/logging.h>

#include <iostream>

namespace MPCPlanner
{

    Planner::Planner(const YAML::Node &config) : _config(config)
    {

        LOG_VALUE("N", _config["N"].as<int>());

        // Initialize the solver
        _solver = std::make_shared<Solver>();
        _solver->reset();

        // Initialize modules
        _modules.emplace_back(nullptr);
        _modules.back() = std::make_shared<MPCBaseModule>(_solver, &_config);
        _modules.emplace_back(nullptr);
        _modules.back() = std::make_shared<GoalModule>(_solver, &_config);
    }

    // Given real-time data, solve the MPC problem
    void Planner::solveMPC(const State &state, const RealTimeData &data)
    {
        // Check if all modules have enough data
        bool is_data_ready = true;
        for (auto &module : _modules)
            is_data_ready = is_data_ready & module->isDataReady(data);

        if (!is_data_ready)
        {
            LOG_WARN("Data is not ready");
            return;
        }

        // Set the initial guess
        _solver->setXinit("x", state.get("x"));
        _solver->setXinit("y", state.get("y"));
        _solver->setXinit("psi", state.get("psi"));
        _solver->setXinit("v", state.get("v"));
        LOG_INFO("Initial guess: " << state.get("x") << ", " << state.get("y") << ", " << state.get("psi") << ", " << state.get("v"));

        // Update all modules
        for (auto &module : _modules)
            module->update(data);

        // Load parameters
        for (int k = 0; k < _solver->N; k++)
        {
            for (auto &module : _modules)
                module->setParameters(data, k);
        }

        // Solve MPC
        int exit_flag = _solver->solve();
        LOG_VALUE("Exit Flag", exit_flag);

        // Verify that the solution is valid
        assert(exit_flag == 1);

        // Print output
        LOG_INFO(_solver->getOutput(0, "a") << ", " << _solver->getOutput(0, "w") << ", " << _solver->getOutput(1, "x") << ", " << _solver->getOutput(1, "y"));

        // Return the solution
        // return _solver;
    }

    double Planner::getSolution(int k, std::string &&var_name)
    {
        return _solver->getOutput(k, std::forward<std::string>(var_name));
    }

};