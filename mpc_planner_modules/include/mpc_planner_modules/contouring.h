#ifndef __CONTOURING_H_
#define __CONTOURING_H_

#include <mpc_planner_modules/controller_module.h>

#include <ros_tools/spline.h>

namespace MPCPlanner
{
  class Contouring : public ControllerModule
  {
  public:
    Contouring(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
    void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    void onDataReceived(RealTimeData &data, std::string &&data_name) override;
    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    bool isObjectiveReached(const State &state, const RealTimeData &data) override;

    void visualize(const RealTimeData &data, const ModuleData &module_data) override;

    void reset() override;

  protected:
    std::shared_ptr<RosTools::Spline2D> _spline{nullptr};
    std::unique_ptr<RosTools::Spline2D> _bound_left{nullptr}, _bound_right{nullptr};

    int _closest_segment{0};
    int _n_segments;

    bool _add_road_constraints{false}, _two_way_road{false}, _dynamic_velocity_reference{false};

    void constructRoadConstraints(const RealTimeData &data, ModuleData &module_data);
    void constructRoadConstraintsFromCenterline(const RealTimeData &data, ModuleData &module_data);
    void constructRoadConstraintsFromBounds(const RealTimeData &data, ModuleData &module_data);

    void setSplineParameters(int k);

    void visualizeReferencePath(const RealTimeData &data, const ModuleData &module_data);
    void visualizeCurrentSegment(const RealTimeData &data, const ModuleData &module_data);
    void visualizeTrackedSection(const RealTimeData &data, const ModuleData &module_data);
    void visualizeRoadConstraints(const RealTimeData &data, const ModuleData &module_data);

    void visualizeDebugRoadBoundary(const RealTimeData &data, const ModuleData &module_data);
    void visualizeDebugGluedSplines(const RealTimeData &data, const ModuleData &module_data);
    void visualizeAllSplineIndices(const RealTimeData &data, const ModuleData &module_data);
  };
}
#endif // __CONTOURING_H_