#ifndef __MPC_GOAL_MODULE_H__
#define __MPC_GOAL_MODULE_H__

#include <mpc_planner_modules/controller_module.h>

namespace MPCPlanner
{
  class GoalModule : public ControllerModule
  {
  public:
    GoalModule(std::shared_ptr<Solver> solver);

  public:
    virtual void update(State &state, const RealTimeData &data, ModuleData &module_data) override;

    virtual void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    bool isObjectiveReached(const State &state, const RealTimeData &data) override;

    // Testing
    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    void visualize(const RealTimeData &data, const ModuleData &module_data) override;

  private:
  };
}

#endif // __MPC_GOAL_MODULE_H__