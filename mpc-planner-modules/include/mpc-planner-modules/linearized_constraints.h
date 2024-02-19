#ifndef __LINEARIZED_CONSTRAINTS_H_
#define __LINEARIZED_CONSTRAINTS_H_

#include <mpc-planner-types/controller_module.h>

namespace MPCPlanner
{
  class LinearizedConstraints : public ControllerModule
  {
  public:
    LinearizedConstraints(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data) override;
    void setParameters(const RealTimeData &data, int k) override;

    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    void visualize(const RealTimeData &data) override;

  private:
    std::vector<std::vector<Eigen::ArrayXd>> _a1, _a2, _b; // Constraints [disc x step]

    // RosTools::DouglasRachford dr_projection_;

    int _num_obstacles;
  };
};
#endif // __LINEARIZED_CONSTRAINTS_H_
