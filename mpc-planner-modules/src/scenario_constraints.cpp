#include "mpc-planner-modules/scenario_constraints.h"

#include <mpc-planner-util/parameters.h>

#include <ros_tools/visuals.h>
#include <ros_tools/math.h>

#include <algorithm>

namespace MPCPlanner
{
  ScenarioConstraints::ScenarioConstraints(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "gaussian_constraints")
  {
    LOG_INITIALIZE("Scenario Constraints");

    _SCENARIO_CONFIG.Init();
    _scenario_module = std::make_unique<ScenarioModule::ScenarioModule>(solver);

    LOG_INITIALIZED();
  }

  void ScenarioConstraints::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;

    _scenario_module->update(data);
  }

  void ScenarioConstraints::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    _scenario_module->setParameters(data, k);
  }

  int ScenarioConstraints::optimize(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;
    (void)data;
    // if (!config_->use_trajectory_sampling_)                        // To test regular optimization with slack
    // return SimpleSequentialScenarioIterations(solver_interface); // S-MPCC (SQP)

    _solver->_params.solver_timeout = 1. / CONFIG["control_frequency"].as<double>();
    // int exit_code = _solver->solve();
    int exit_code = _scenario_module->optimize(data); // Safe Horizon MPC
    return exit_code;
  }

  void ScenarioConstraints::onDataReceived(RealTimeData &data, std::string &&data_name)
  {
    LOG_DEBUG("ScenarioConstraints::OnDataReceived()");

    if (data_name == "dynamic obstacles")
    {
      if (_SCENARIO_CONFIG.enable_safe_horizon_)
      {
        _scenario_module->GetSampler().IntegrateAndTranslateToMeanAndVariance(data.dynamic_obstacles, _solver->dt);
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

    if (!_scenario_module->isDataReady(data, missing_data))
      return false;

    return true;
  }

  void ScenarioConstraints::visualize(const RealTimeData &data, const ModuleData &module_data)
  {
    LOG_DEBUG("ScenarioConstraints::visualize");
    _scenario_module->visualize(data);
  }

} // namespace MPCPlanner
