#ifndef __CONTOURING_H_
#define __CONTOURING_H_

#include <mpc-planner-types/controller_module.h>

#include <mpc-planner-util/spline.h>

// #include <ros_tools/helpers.h>

// #include <Eigen/Dense>

// Whens earching for the closest point on the path, this variable indicates the distance that the algorithm searches
// behind the current spline point.
// #define MAX_STEP_BACK_TOLERANCE 0.1f

namespace MPCPlanner
{
  class Contouring : public ControllerModule
  {
  public:
    Contouring(std::shared_ptr<Solver> solver);

  public:
    // Outputs
    // unsigned int spline_index_{0}; // Where we are on the spline
    // bool ready_{false};

    // bool goal_reached_{false};
    // bool first_run_{true};

    void update(State &state, const RealTimeData &data) override;
    void setParameters(const RealTimeData &data, int k) override;

    void onDataReceived(RealTimeData &data, std::string &&data_name) override;
    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    bool isObjectiveReached(const RealTimeData &data) override;

    void visualize(const RealTimeData &data) override;

  private:
    std::unique_ptr<Spline2D> _spline{nullptr};
  };
};
#endif // __CONTOURING_H_

/**
  public:
    // void OnReset(SolverInterface *solver_interface) override;
    // void OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name) override;


void InitPath();

void InitPath(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &psi);


void InitPath(const std::vector<double> &x, const std::vector<double> &y);


// void UpdateClosestPoint(SolverInterface *solver_interface_ptr, double &s_guess, double window = 2, int n_tries = 20);

/**
 * @brief Find the closest spline segment in the reference path
 *
 * @param solver_interface_ptr solver interface to retrieve the vehicle position from
 */
// double InitializeClosestPoint(SolverInterface *solver_interface_ptr);

/**
 * @brief Is the current spline segment at an end?
 *
 * @param index Current spline index
 * @return true If end of current spline segment was reached
 */
// bool EndOfCurrentSpline(double index);

/**
 * @brief Checks if the end of the path was reached
 *
 * @return true If end of the path was reached
 */
// bool ReachedEnd();

/**
 * @brief Construct linear road constraints by linearizing the spline boundaries w.r.t. the vehicle position
 *
 * @param solver_interface solver interface to retrieve the vehicle positions from
 * @param halfspaces_out Output halfspaces as vector with entries for each time step
 */
// void ConstructRoadConstraints(SolverInterface *solver_interface, std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out);

/**
 * @brief Construct linear road constraints by linearizing the received boundaries
 */
// void ConstructRoadConstraintsFromData(SolverInterface *solver_interface,
// std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out);
// Visualization
/** @brief Draw the reference path (line) and its orientations (arrows)*/
// void PublishReferencePath();

/** @brief Publish cubes marking the current spline index followed by the vehicle  */
// void PublishCurrentSplineIndex();

/**
 * @brief Visualize the linearized road boundary constraints (the ones gives) - Debug functionality only
 *
 * @param solver_interface_ptr solver
 * @param halfspaces_out the constraints to plot
 */
// void PublishLinearRoadBoundaries(SolverInterface *solver_interface_ptr,
//  const std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out);

/**
 * @brief Visualize road boundaries as a spline (i.e., not linearized)
 *
 * @param solver_interface_ptr solver interface
 */
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

// /** @brief Read a reference path from ROS Parameters */
// void ReadReferencePath(std::vector<double> &x_out, std::vector<double> &y_out);

// void ProcessReceivedReferencePath(const PathWithBounds &path);
// /**   @brief Construct a reference path from a set of waypoints (x, y) */
// void ConstructReferencePath(const PathWithBounds &path); // Selects which method to use

// // void WaypointsToReferencePath(const std::vector<double> &x, const std::vector<double> &y);

// // void FitRoadBoundaries(const std::vector<double> &x_left, const std::vector<double> &x_right,
// //                        const std::vector<double> &y_left, const std::vector<double> &y_right);

// void ComputeDistanceVector(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &out);

// /**
//  * @brief Search for the closest point to the vehicle
//  *
//  * @param solver_interface_ptr Solver interface to retrieve the vehicle position
//  * @param cur_traj_i Current spline index
//  * @param s_guess Guessed spline value
//  * @param window Window to search in w.r.t. the guess
//  * @param n_tries Number of recursions allowed
//  * @return int The closest spline index
//  */
// int RecursiveClosestPointSearch(SolverInterface *solver_interface_ptr, unsigned int cur_traj_i, double &s_guess,
//                                 double window, int n_tries, int num_recursions);

// double FindClosestSRecursively(const Eigen::Vector2d &pose, double low, double high, int num_recursions);

// void OnWaypointsReceived();
// }
// ;
// * /
// }
// ;
