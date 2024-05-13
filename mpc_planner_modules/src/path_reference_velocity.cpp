
#include <mpc_planner_modules/path_reference_velocity.h>

#include <mpc_planner_generated.h>
#include <mpc_planner_util/parameters.h>

namespace MPCPlanner
{

  PathReferenceVelocity::PathReferenceVelocity(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "path_reference_velocity")
  {
  }

  void PathReferenceVelocity::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;
    (void)data;
    (void)module_data;
  }

  void PathReferenceVelocity::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)module_data;
    (void)data;

    static double velocity_weight, reference_velocity;

    // Retrieve once
    if (k == 0)
    {
      velocity_weight = CONFIG["weights"]["velocity"].as<double>();
      reference_velocity = CONFIG["weights"]["reference_velocity"].as<double>();
    }

    // Set the parameters for velocity tracking
    setForcesParameterVelocity(k, _solver->_params, velocity_weight);
    setForcesParameterReferenceVelocity(k, _solver->_params, reference_velocity);
  }
} // namespace MPCPlanner