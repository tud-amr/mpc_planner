#ifndef __CONTOURING_CONSTRAINTS_H_
#define __CONTOURING_CONSTRAINTS_H_

#include <mpc_planner_modules/controller_module.h>

namespace tk
{
  class spline;
}

namespace MPCPlanner
{
  class ContouringConstraints : public ControllerModule
  {
  public:
    ContouringConstraints(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
    void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    void onDataReceived(RealTimeData &data, std::string &&data_name) override;

    void visualize(const RealTimeData &data, const ModuleData &module_data) override;

  private:
    int _num_segments;

    std::unique_ptr<tk::spline> _width_left{nullptr}, _width_right{nullptr};
  };
}
#endif // __ELLIPSOID_CONSTRAINTS_H_
