
#include <mpc-planner-modules/mpc_base.h>

#include <mpc-planner-modules/definitions.h>

#include <mpc-planner-util/parameters.h>

namespace MPCPlanner
{

  MPCBaseModule::MPCBaseModule(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "mpc_base")
  {
    _weight_names = WEIGHT_PARAMS;
  }

  void MPCBaseModule::update(State &state, const RealTimeData &data)
  {
    (void)state;
    (void)data;
  }

  void MPCBaseModule::setParameters(const RealTimeData &data, int k)
  {
    (void)data;
    if (k == 0)
      LOG_DEBUG("setParameters()");

    for (auto &weight : _weight_names)
    {
      _solver->setParameter(k, weight, CONFIG["weights"][weight].as<double>());
    }
  }
} // namespace MPCPlanner