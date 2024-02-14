
#include <mpc-planner/mpc_base.h>

namespace MPCPlanner
{

  MPCBaseModule::MPCBaseModule(std::shared_ptr<Solver> solver, YAML::Node *config)
      : ControllerModule(solver, config, ModuleType::OBJECTIVE, "mpc_base")
  {
  }

  void MPCBaseModule::update(const RealTimeData &data)
  {
  }

  void MPCBaseModule::setParameters(const RealTimeData &data, int k)
  {
    if (k == 0)
      LOG_INFO("SetParameters()");

    // Set the parameters for the solver
    _solver->setParameter(k, "acceleration", CONFIG("acceleration_weight").as<double>());
    _solver->setParameter(k, "angular_velocity", CONFIG("angular_velocity_weight").as<double>());
  }
};