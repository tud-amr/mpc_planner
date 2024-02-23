#ifndef __GAUSSIAN_CONSTRAINTS_H_
#define __GAUSSIAN_CONSTRAINTS_H_

#include <mpc-planner-modules/controller_module.h>

namespace MPCPlanner
{
  class GaussianConstraints : public ControllerModule
  {
  public:
    GaussianConstraints(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data) override;
    void setParameters(const RealTimeData &data, int k) override;

    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    void visualize(const RealTimeData &data) override;

  private:
  };
}
#endif // __GAUSSIAN_CONSTRAINTS_H_
