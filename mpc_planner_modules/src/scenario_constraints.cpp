#include "mpc_planner_modules/scenario_constraints.h"

#include <mpc_planner_util/data_visualization.h>
#include <mpc_planner_util/parameters.h>

#include <ros_tools/visuals.h>
#include <ros_tools/math.h>
#include <ros_tools/profiling.h>

#include <algorithm>

#include <omp.h>

namespace MPCPlanner
{

  ScenarioConstraints::ScenarioSolver::ScenarioSolver(int id)
  {
    solver = std::make_shared<Solver>(id);
    scenario_module.initialize(solver);
  }

  ScenarioConstraints::ScenarioConstraints(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "scenario_constraints")
  {
    LOG_INITIALIZE("Scenario Constraints");

    _planning_time = 1. / CONFIG["control_frequency"].as<double>();

    _SCENARIO_CONFIG.Init();
    for (int i = 0; i < CONFIG["scenario_constraints"]["parallel_solvers"].as<int>(); i++)
    {
      _scenario_solvers.emplace_back(std::make_unique<ScenarioSolver>(i)); // May need an integer input
    }
    LOG_INITIALIZED();
  }

  void ScenarioConstraints::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;

#pragma omp parallel for num_threads(4)
    for (auto &solver : _scenario_solvers)
    {
      *solver->solver = *_solver; // Copy the main solver, including its initial guess

      solver->scenario_module.update(data, module_data);
    }
  }

  void ScenarioConstraints::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)module_data;
    (void)data;
    (void)k;
  }

  int ScenarioConstraints::optimize(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;
    (void)module_data;

    omp_set_nested(1);
    omp_set_max_active_levels(2);
    omp_set_dynamic(0);

#pragma omp parallel for num_threads(4)
    for (auto &solver : _scenario_solvers)
    {
      // Set the planning timeout
      std::chrono::duration<double> used_time = std::chrono::system_clock::now() - data.planning_start_time;
      solver->solver->_params.solver_timeout = _planning_time - used_time.count() - 0.008;

      // Copy solver parameters and initial guess
      *solver->solver = *_solver; // Copy the main solver

      // Set the scenario constraint parameters for each solver
      for (int k = 0; k < _solver->N; k++)
      {
        solver->scenario_module.setParameters(data, k);
      }

      solver->solver->loadWarmstart(); // Load the previous solution

      solver->exit_code = solver->scenario_module.optimize(data); // Safe Horizon MPC
    }
    omp_set_dynamic(1);

    // Retrieve the lowest cost solution
    double lowest_cost = 1e9;
    _best_solver = nullptr;
    for (auto &solver : _scenario_solvers)
    {
      if (solver->exit_code == 1 && solver->solver->_info.pobj < lowest_cost)
      {
        lowest_cost = solver->solver->_info.pobj;
        _best_solver = solver.get();
      }
    }
    if (_best_solver == nullptr) // No feasible solution
      return _scenario_solvers.front()->exit_code;

    _solver->_output = _best_solver->solver->_output; // Load the solution into the main lmpcc solver
    _solver->_info = _best_solver->solver->_info;
    _solver->_params = _best_solver->solver->_params;

    return _best_solver->exit_code;
  }

  void ScenarioConstraints::onDataReceived(RealTimeData &data, std::string &&data_name)
  {
    LOG_DEBUG("ScenarioConstraints::OnDataReceived()");

    if (data_name == "dynamic obstacles")
    {
      if (_SCENARIO_CONFIG.enable_safe_horizon_)
      {
#pragma omp parallel for num_threads(4)
        for (auto &solver : _scenario_solvers) // Draw different samples for all solvers
        {
          solver->scenario_module.GetSampler().IntegrateAndTranslateToMeanAndVariance(data.dynamic_obstacles, _solver->dt);
        }
      }
    }
  }

  bool ScenarioConstraints::isDataReady(const RealTimeData &data, std::string &missing_data)
  {

    if (data.dynamic_obstacles.size() != CONFIG["max_obstacles"].as<unsigned int>())
    {
      missing_data += "Obstacles ";
      return false;
    }

    for (size_t i = 0; i < data.dynamic_obstacles.size(); i++)
    {
      if (data.dynamic_obstacles[i].prediction.empty())
      {
        missing_data += "Obstacle Prediction ";
        return false;
      }

      if (data.dynamic_obstacles[i].prediction.type == PredictionType::DETERMINISTIC)
      {
        missing_data += "Uncertain Predictions (scenario-based control cannot use deterministic predictions) ";
        return false;
      }
    }

    if (!_scenario_solvers.front()->scenario_module.isDataReady(data, missing_data))
      return false;

    return true;
  }

  void ScenarioConstraints::visualize(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)module_data;
    bool visualize_all = false;

    LOG_MARK("ScenarioConstraints::visualize");

    if (visualize_all)
    {
      for (auto &solver : _scenario_solvers)
        solver->scenario_module.visualize(data);
    }
    else if (_best_solver != nullptr)
    {

      _best_solver->scenario_module.visualize(data);
    }

    // Visualize optimized trajectories
    for (auto &solver : _scenario_solvers)
    {
      if (solver->exit_code == 1)
      {
        Trajectory trajectory;
        for (int k = 1; k < _solver->N; k++)
          trajectory.add(solver->solver->getOutput(k, "x"), solver->solver->getOutput(k, "y"));

        visualizeTrajectory(trajectory, _name + "/optimized_trajectories", false, 0.2, solver->solver->_solver_id, 2 * _scenario_solvers.size());
      }
    }
    VISUALS.getPublisher(_name + "/optimized_trajectories").publish();
  }

} // namespace MPCPlanner
