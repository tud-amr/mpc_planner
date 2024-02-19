
#include <mpc-planner-modules/mpc_base.h>

#include <mpc-planner-util/parameters.h>

namespace MPCPlanner
{

  MPCBaseModule::MPCBaseModule(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "mpc_base")
  {
  }

  void MPCBaseModule::update(State &state, const RealTimeData &data)
  {
  }

  void MPCBaseModule::setParameters(const RealTimeData &data, int k)
  {
    if (k == 0)
      LOG_DEBUG("setParameters()");

    // Set the parameters for the solver
    _solver->setParameter(k, "acceleration", CONFIG["weights"]["acceleration"].as<double>());
    _solver->setParameter(k, "angular_velocity", CONFIG["weights"]["angular_velocity"].as<double>());
    _solver->setParameter(k, "velocity", CONFIG["weights"]["velocity"].as<double>());
    _solver->setParameter(k, "reference_velocity", CONFIG["reference_velocity"].as<double>());
  }
};