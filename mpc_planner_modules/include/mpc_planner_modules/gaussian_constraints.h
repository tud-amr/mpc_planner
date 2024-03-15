#ifndef __GAUSSIAN_CONSTRAINTS_H_
#define __GAUSSIAN_CONSTRAINTS_H_

#include <mpc_planner_modules/controller_module.h>

namespace MPCPlanner
{
  class GaussianConstraints : public ControllerModule
  {
  public:
    GaussianConstraints(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
    void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    void visualize(const RealTimeData &data, const ModuleData &module_data) override;

  private:
  };
}
#endif // __GAUSSIAN_CONSTRAINTS_H_
