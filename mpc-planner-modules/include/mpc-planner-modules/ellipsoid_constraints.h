#ifndef __ELLIPSOID_CONSTRAINTS_H_
#define __ELLIPSOID_CONSTRAINTS_H_

#include <mpc-planner-modules/controller_module.h>

namespace MPCPlanner
{
  class EllipsoidConstraints : public ControllerModule
  {
  public:
    EllipsoidConstraints(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
    void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    // void onDataReceived(RealTimeData &data, std::string &&data_name) override;

    void visualize(const RealTimeData &data, const ModuleData &module_data) override;

  private:
  };
}
#endif // __ELLIPSOID_CONSTRAINTS_H_
