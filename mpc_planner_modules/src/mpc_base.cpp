#include <mpc_planner_modules/mpc_base.h>

#include <mpc_planner_modules/definitions.h>

#include <mpc_planner_util/parameters.h>

namespace MPCPlanner
{

  MPCBaseModule::MPCBaseModule(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "mpc_base")
  {
    _weight_names = WEIGHT_PARAMS;
  }

  void MPCBaseModule::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;
    (void)data;
    (void)module_data;
  }

  void MPCBaseModule::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)data;
    (void)module_data;

    if (k == 0)
      LOG_MARK("setParameters()");

    for (auto &weight : _weight_names)
    {
      _solver->setParameter(k, weight, CONFIG["weights"][weight].as<double>());
    }
  }
} // namespace MPCPlanner