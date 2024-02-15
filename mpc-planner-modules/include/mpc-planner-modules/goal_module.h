#ifndef __MPC_GOAL_MODULE_H__
#define __MPC_GOAL_MODULE_H__

#include <mpc-planner-types/controller_module.h>

namespace MPCPlanner
{
  class GoalModule : public ControllerModule
  {
  public:
    GoalModule(std::shared_ptr<Solver> solver);

  public:
    virtual void update(const RealTimeData &data) override;

    virtual void setParameters(const RealTimeData &data, int k) override;

    // Testing
    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    void visualize(const RealTimeData &data) override;

  private:
  };
};

#endif // __MPC_GOAL_MODULE_H__