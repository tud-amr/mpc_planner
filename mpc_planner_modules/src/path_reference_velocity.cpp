
#include <mpc_planner_modules/path_reference_velocity.h>

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

    // Set the parameters for velocity tracking
    _solver->setParameter(k, "velocity", CONFIG["weights"]["velocity"].as<double>());
    _solver->setParameter(k, "reference_velocity", CONFIG["weights"]["reference_velocity"].as<double>());
  }
} // namespace MPCPlanner