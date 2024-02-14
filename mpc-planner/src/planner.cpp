#include <mpc-planner/planner.h>

#include <mpc-planner-types/controller_module.h>
#include <mpc-planner-types/realtime_data.h>
#include <mpc-planner-solver/solver_interface.h>

/** @todo Generate */
#include <mpc-planner/mpc_base.h>
#include <mpc-planner/goal_module.h>

#include <mpc-planner-util/load_yaml.hpp>
#include <mpc-planner-util/logging.h>
#include <mpc-planner-util/visuals.h>

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
    bool Planner::solveMPC(const State &state, const RealTimeData &data)
    {
        bool success = true;

        // Check if all modules have enough data
        bool is_data_ready = true;
        for (auto &module : _modules)
            is_data_ready = is_data_ready & module->isDataReady(data);

        if (!is_data_ready)
        {
            LOG_WARN("Data is not ready");
            return false;
        }

        // Set the initial guess
        _solver->setXinit(state);

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

        if (exit_flag != 1)
        {
            success = false;
            LOG_WARN("MPC did not find a solution");
            LOG_VALUE("Exit Flag", exit_flag); /** @todo: Convertion to text */
        }

        // Verify that the solution is valid
        assert(exit_flag == 1);

        return success;
    }

    double Planner::getSolution(int k, std::string &&var_name)
    {
        return _solver->getOutput(k, std::forward<std::string>(var_name));
    }

    void Planner::visualize(const State &state, const RealTimeData &data)
    {
        
        for (auto &module : _modules)
            module->visualize(data);

        // Visualize the trajectory
        auto &publisher = VISUALS.getPublisher("planned_trajectory");

        auto &cylinder = publisher.getNewPointMarker("CYLINDER");
        cylinder.setScale(2. * _config["robot_radius"].as<double>(), 2. * _config["robot_radius"].as<double>(), 0.01); // Replace with robot size
        cylinder.setColorInt(0, 0.4);

        Trajectory output_trajectory(_solver->dt, _solver->N);
        for (int k = 0; k < _solver->N; k++)
            output_trajectory.add(_solver->getOutput(k, "x"), _solver->getOutput(k, "y"));

        for (int k = 0; k < _solver->N; k++)
            cylinder.addPointMarker(output_trajectory.position[k]);

        publisher.publish();
    }

};