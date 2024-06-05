#ifndef __DECOMP_CONSTRAINTS_H_
#define __DECOMP_CONSTRAINTS_H_

#include <mpc_planner_modules/controller_module.h>

#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/decomp_geometry/geometric_utils.h>

#include <ros_tools/projection.h>

namespace MPCPlanner
{
  class DecompConstraints : public ControllerModule
  {
  public:
    DecompConstraints(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
    void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    void visualize(const RealTimeData &data, const ModuleData &module_data) override;

  private:
    std::vector<std::vector<Eigen::ArrayXd>> _a1, _a2, _b; // Constraints [disc x step]

    std::unique_ptr<EllipsoidDecomp2D> _decomp_util;
    vec_Vec2f _occ_pos;
    std::vector<LinearConstraint<2>> _constraints; // Static 2D halfspace constraints set in DecompUtil
    vec_E<Polyhedron<2>> _polyhedrons;
    std::vector<std::unique_ptr<vec_Vec2f>> occ_pos_vec_stages_;

    double _dummy_a1{1.}, _dummy_a2{0.}, _dummy_b;

    int _n_discs;

    DouglasRachford dr_projection_;

    int _max_constraints;

    bool getOccupiedGridCells(const RealTimeData &data);

    void projectToSafety(Eigen::Vector2d &pos);
  };
} // namespace MPCPlanner
#endif // __DECOMP_CONSTRAINTS_H_
