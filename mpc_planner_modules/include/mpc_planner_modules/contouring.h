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

  private:
    std::shared_ptr<RosTools::Spline2D> _spline{nullptr};
    std::unique_ptr<RosTools::Spline2D> _bound_left{nullptr}, _bound_right{nullptr};

    int _closest_segment{0};
    int _n_segments;

    bool _add_road_constraints{false}, _two_way_road{false}, _use_ca_mpc;

    void constructRoadConstraints(const RealTimeData &data, ModuleData &module_data);
    void constructRoadConstraintsFromCenterline(const RealTimeData &data, ModuleData &module_data);
    void constructRoadConstraintsFromBounds(const RealTimeData &data, ModuleData &module_data);

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

/**
  public:
    // void OnReset(SolverInterface *solver_interface) override;
    // void OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name) override;


void InitPath();

void InitPath(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &psi);


void InitPath(const std::vector<double> &x, const std::vector<double> &y);


// void UpdateClosestPoint(SolverInterface *solver_interface_ptr, double &s_guess, double window = 2, int n_tries = 20);

// double InitializeClosestPoint(SolverInterface *solver_interface_ptr);

// bool EndOfCurrentSpline(double index);

// bool ReachedEnd();

// void ConstructRoadConstraints(SolverInterface *solver_interface, std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out);

// void ConstructRoadConstraintsFromData(SolverInterface *solver_interface,
// std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out);
// Visualization
// void PublishReferencePath();

// void PublishCurrentSplineIndex();

// void PublishLinearRoadBoundaries(SolverInterface *solver_interface_ptr,
//  const std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out);

// void VisualizeRoad();

// protected:
// // Inputs
// // std::vector<double> waypoints_x_, waypoints_y_; // Waypoints
// // PathWithBounds received_reference_path_;

// // Processing
// double current_s_;             // At what distance we are along the spline
// std::vector<double> s_vector_; // s_vector holds the distance at which each spline begins (i.e., at the end of each spline)

// // ROS Parameters
// std::vector<double> ref_x_, ref_y_, ref_theta_; // X and Y points from yaml

// double road_width_right_, road_width_left_;
// bool enable_two_way_road_, enable_road_boundary_linearization_;
// // int n_points_clothoid_, n_points_spline_;

// // double min_wapoints_distance_;
// // double epsilon_;

// void ReadReferencePath(std::vector<double> &x_out, std::vector<double> &y_out);

// void ProcessReceivedReferencePath(const PathWithBounds &path);
// void ConstructReferencePath(const PathWithBounds &path); // Selects which method to use

// // void WaypointsToReferencePath(const std::vector<double> &x, const std::vector<double> &y);

// // void FitRoadBoundaries(const std::vector<double> &x_left, const std::vector<double> &x_right,
// //                        const std::vector<double> &y_left, const std::vector<double> &y_right);

// void ComputeDistanceVector(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &out);

// int RecursiveClosestPointSearch(SolverInterface *solver_interface_ptr, unsigned int cur_traj_i, double &s_guess,
//                                 double window, int n_tries, int num_recursions);

// double FindClosestSRecursively(const Eigen::Vector2d &pose, double low, double high, int num_recursions);

// void OnWaypointsReceived();
// }
// ;
// }
// ;
*/