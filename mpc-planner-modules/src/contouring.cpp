#include "mpc-planner-modules/contouring.h"

#include <mpc-planner-util/parameters.h>
#include <mpc-planner-util/visuals.h>

#include <algorithm>

namespace MPCPlanner
{
  Contouring::Contouring(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "contouring")
  {
    LOG_INFO("Initializing Contouring Module");
  }

  void Contouring::update(State &state, const RealTimeData &data)
  {
    (void)data;

    LOG_DEBUG("contouring::update()");
    if (_spline.get() == nullptr)
    {
      LOG_WARN("Not updating yet");
      return;
    }
    // Get the closest point
    double closest_s;

    // Update the closest point
    _spline->findClosestPoint(Eigen::Vector2d(state.get("x"), state.get("y")), _closest_segment, closest_s);

    state.set("spline", closest_s); // We need to initialize the spline state here
  }

  void Contouring::setParameters(const RealTimeData &data, int k)
  {
    (void)data;
    LOG_DEBUG("contouring::setparameters");

    _solver->setParameter(k, "contour", CONFIG["contouring"]["contour"].as<double>());
    _solver->setParameter(k, "lag", CONFIG["contouring"]["lag"].as<double>());

    /** @todo: Handling of parameters when the spline parameters go beyond the splines defined */
    for (int i = 0; i < CONFIG["contouring"]["num_segments"].as<int>(); i++)
    {
      int index = _closest_segment + i;
      double ax, bx, cx, dx;
      double ay, by, cy, dy;
      double start;

      if (index < _spline->numSegments())
      {
        _spline->getParameters(index,
                               ax, bx, cx, dx,
                               ay, by, cy, dy);

        start = _spline->getStartOfSegment(index);
      }
      else
      {
        LOG_WARN("Beyond the spline");
        // If we are beyond the spline, we should use the last spline
        _spline->getParameters(_spline->numSegments() - 1,
                               ax, bx, cx, dx,
                               ay, by, cy, dy);

        start = _spline->getStartOfSegment(_spline->numSegments() - 1);

        // We should use very small splines at the end location
        // x = dx
        // y = dy
        ax = 0.;
        bx = 0.;
        cx = 0.;
        ay = 0.;
        by = 0.;
        cy = 0.;
        start = _spline->length();
      }

      std::string spline_name = "spline" + std::to_string(i) + "_";

      _solver->setParameter(k, spline_name + "ax", ax);
      _solver->setParameter(k, spline_name + "bx", bx);
      _solver->setParameter(k, spline_name + "cx", cx);
      _solver->setParameter(k, spline_name + "dx", dx);

      _solver->setParameter(k, spline_name + "ay", ay);
      _solver->setParameter(k, spline_name + "by", by);
      _solver->setParameter(k, spline_name + "cy", cy);
      _solver->setParameter(k, spline_name + "dy", dy);

      // Distance where this spline starts
      _solver->setParameter(k, spline_name + "start", start);
    }
  }

  void Contouring::onDataReceived(RealTimeData &data, std::string &&data_name)
  {
    if (data_name == "reference_path")
    {
      LOG_INFO("Received Reference Path");

      _spline = std::make_unique<Spline2D>(data.reference_path.x, data.reference_path.y); // Construct a spline from the given points
    }
  }

  bool Contouring::isDataReady(const RealTimeData &data, std::string &missing_data)
  {
    if (data.reference_path.x.empty())
      missing_data += "Reference Path ";

    return !data.reference_path.x.empty();
  }

  bool Contouring::isObjectiveReached(const RealTimeData &data)
  {
    (void)data;
    return false;
  }

  void Contouring::visualize(const RealTimeData &data)
  {
    if (_spline.get() == nullptr)
      return;

    // Visualize the current points
    auto &publisher_current = VISUALS.getPublisher(_name + "/current");
    auto &cur_point = publisher_current.getNewPointMarker("CUBE");
    cur_point.setColorInt(10);
    cur_point.setScale(0.3, 0.3, 0.3);
    cur_point.addPointMarker(_spline->getPoint(_spline->getStartOfSegment(_closest_segment)), 0.0);
    publisher_current.publish();

    // Visualize the points
    auto &publisher_points = VISUALS.getPublisher(_name + "/points");
    auto &point = publisher_points.getNewPointMarker("CYLINDER");
    point.setColor(0., 0., 0.);
    point.setScale(0.15, 0.15, 0.05);

    for (size_t p = 0; p < data.reference_path.x.size(); p++)
      point.addPointMarker(Eigen::Vector3d(data.reference_path.x[p], data.reference_path.y[p], 0.1));
    publisher_points.publish();

    // Visualize the path
    auto &publisher_path = VISUALS.getPublisher(_name + "/path");
    auto &line = publisher_path.getNewLine();
    line.setColorInt(5);
    line.setScale(0.1);

    Eigen::Vector2d p;
    for (double s = 0.; s < _spline->length(); s += 1.)
    {
      if (s > 0.)
        line.addLine(p, _spline->getPoint(s));

      p = _spline->getPoint(s);
    }

    publisher_path.publish();
  }

  void Contouring::reset()
  {
    _spline.reset(nullptr);

    // goal_reached_ = false;
  }

} // namespace MPCPlanner

// Contouring::Contouring(rclcpp::Node *node, std::shared_ptr<SolverInterface> solver, MPCConfiguration *config, VehicleRegion *vehicle, ModuleType type, std::string &&module_name)
//     : ControllerModule(node, solver, config, vehicle, type, std::forward<std::string>(module_name))
// {
//   LMPCC_INFO_ALWAYS(logger_, "Initializing Reference Path");

//   // Reference Path
//   ros_markers_reference_path_.reset(new RosTools::ROSMarkerPublisher(node, config_->reference_path_topic_.c_str(), config_->target_frame_, 45));
//   ros_markers_reference_arrows_.reset(new RosTools::ROSMarkerPublisher(node, config_->reference_arrows_topic_.c_str(), config_->target_frame_, 30));
//   ros_markers_splineindex.reset(new RosTools::ROSMarkerPublisher(node, config_->spline_index_topic_.c_str(), config_->target_frame_, 5));
//   ros_markers_linearboundaries.reset(new RosTools::ROSMarkerPublisher(node, "lmpcc/linear_road_constraints", config_->target_frame_, 100));
//   ros_markers_road_limits.reset(new RosTools::ROSMarkerPublisher(node, "lmpcc/road_limits", config_->target_frame_, 300));

//   DeclareROSParameters(node);

//   // Read the reference from ROS Parameters
//   ReadReferencePath(received_reference_path_.centerline.x,
//                     received_reference_path_.centerline.y);

//   // Use the read waypoints to construct a reference path
//   ProcessReceivedReferencePath(received_reference_path_);

//   LMPCC_SUCCESS_ALWAYS(logger_, "Initialized");
// }

// void Contouring::ProcessReceivedReferencePath(const PathWithBounds &path)
// {
//   // Initialize the path from file
//   ConstructReferencePath(path);

//   // Set the tracking index
//   spline_index_ = InitializeClosestPoint(solver_.get()); // Stable initialization

//   // Initialize the solver spline parameter
//   UpdateClosestPoint(solver_.get(), current_s_); // Recursive initialization
//   solver_->setInitialSpline(current_s_);         // In SetParameters?
//   for (size_t k = 0; k < solver_->FORCES_N; k++)
//     solver_->spline(k) = current_s_;
// }

// void Contouring::ReadReferencePath(std::vector<double> &x_out, std::vector<double> &y_out)
// {
//   LMPCC_INFO_ALWAYS(logger_, "Reading Reference Path");

//   // Check if all reference vectors are of the same length
//   assert(ref_x_.size() == ref_y_.size());
//   assert(ref_x_.size() == ref_theta_.size());

//   geometry_msgs::msg::Pose pose;
//   tf2::Quaternion myQuaternion;

//   // Iterate over the reference points given
//   x_out.resize(ref_x_.size());
//   y_out.resize(ref_y_.size());
//   for (size_t ref_point_it = 0; ref_point_it < ref_x_.size(); ref_point_it++)
//   {
//     // Create a pose at each position
//     pose.position.x = ref_x_.at(ref_point_it);
//     pose.position.y = ref_y_.at(ref_point_it);

//     x_out[ref_point_it] = pose.position.x;
//     y_out[ref_point_it] = pose.position.y;
//   }
// }

// void Contouring::ConstructReferencePath(const PathWithBounds &path)
// {
//   LMPCC_INFO(logger_, "Fitting cubic splines through set of 2D points");

//   auto &centerline = path.centerline;

//   // Compute new distances locally
//   std::vector<double> s_local;
//   s_local.resize(centerline.x.size());

//   ComputeDistanceVector(centerline.x, centerline.y, s_local);

//   // Fit cubic splines
//   tk::spline local_path_x, local_path_y;
//   local_path_x.set_points(s_local, centerline.x);
//   local_path_y.set_points(s_local, centerline.y);

//   // The distances here could be incorrect (they are computed in a straight line!)
//   // Try: Fix it here by accurately computing the distances then fitting again!
//   // std::vector<double> spline_x, spline_y, spline_s;
//   // std::vector<double> temp_s;
//   // int steps = 10;
//   // spline_s.push_back(0.);
//   // for (size_t index = 0; index < s_local.size(); index++)
//   // {
//   //   double s_step = (s_local[index - 1] - s_local[index]) / ((double)(steps - 1));
//   //   double cur_s = s_local[index - 1];
//   //   for (int s = 0; s < steps; s++)
//   //   {
//   //     // Accurate x and y points
//   //     spline_x.push_back(local_path_x(cur_s));
//   //     spline_y.push_back(local_path_y(cur_s));
//   //     cur_s += s_step;
//   //   }
//   //   ComputeDistanceVector(spline_x, spline_y, temp_s); // Compute the distances over this spline
//   //   spline_s.push_back(temp_s.back());                 // Add the final distance
//   // }

//   // s_local = spline_s;
//   // local_path_x.set_points(spline_s, x);
//   // local_path_y.set_points(spline_s, y);

//   reference_path_.reset(new RosTools::CubicSpline2D<tk::spline>(local_path_x, local_path_y));

//   // Fit the road boundaries using the same `s`
//   // May have to verify that this vector for `s` is valid with respect to the center path
//   if (path.hasBounds())
//   {
//     std::cout << "path has bounds, bound size: " << path.left_bound.x.size() << " vs path size: " << s_local.size() << std::endl;
//     tk::spline left_bound_x, left_bound_y;
//     left_bound_x.set_points(s_local, path.left_bound.x);
//     left_bound_y.set_points(s_local, path.left_bound.y);

//     tk::spline right_bound_x, right_bound_y;
//     right_bound_x.set_points(s_local, path.right_bound.x);
//     right_bound_y.set_points(s_local, path.right_bound.y);

//     // Save the new reference path
//     left_bound_.reset(new RosTools::CubicSpline2D<tk::spline>(left_bound_x, left_bound_y));
//     right_bound_.reset(new RosTools::CubicSpline2D<tk::spline>(right_bound_x, right_bound_y));
//   }

//   s_vector_ = s_local; // Copy the distances over
//   ready_ = true;

//   LMPCC_SUCCESS(logger_, "Reference path ready");
// }

// // void Contouring::FitRoadBoundaries(const std::vector<double> &x_left, const std::vector<double> &x_right,
// //                                    const std::vector<double> &y_left, const std::vector<double> &y_right)
// // {
// //   // Fit the road boundaries (same s points, different waypoints)
// //   tk::spline left_path_x, left_path_y;
// //   left_path_x.set_points(s_vector_, x_left);
// //   left_path_y.set_points(s_vector_, y_left);
// //   left_bound_.reset(new RosTools::CubicSpline2D<tk::spline>(left_path_x, left_path_y));

// //   tk::spline right_path_x, right_path_y;
// //   right_path_x.set_points(s_vector_, x_right);
// //   right_path_y.set_points(s_vector_, y_right);
// //   right_bound_.reset(new RosTools::CubicSpline2D<tk::spline>(right_path_x, right_path_y));
// // }

// void Contouring::DeclareROSParameters(rclcpp::Node *node)
// {
//   ref_x_ = DeclareOrGetParam<std::vector<double>>(node, "global_path.x");
//   ref_y_ = DeclareOrGetParam<std::vector<double>>(node, "global_path.y");
//   ref_theta_ = DeclareOrGetParam<std::vector<double>>(node, "global_path.theta");

//   enable_road_boundary_linearization_ = DeclareOrGetParam<bool>(node, "options.enable_road_boundary_linearization");
//   enable_two_way_road_ = DeclareOrGetParam<bool>(node, "options.enable_two_way_road");

//   road_width_right_ = DeclareOrGetParam<double>(node, "road.width_right");
//   road_width_left_ = DeclareOrGetParam<double>(node, "road.width_left");
// }

// rcl_interfaces::msg::SetParametersResult Contouring::UpdateROSParameters(const std::vector<rclcpp::Parameter> &parameters)
// {
//   updateParam<std::vector<double>>(parameters, "global_path.x", ref_x_);
//   updateParam<std::vector<double>>(parameters, "global_path.y", ref_y_);
//   updateParam<std::vector<double>>(parameters, "global_path.theta", ref_theta_);
//   updateParam<bool>(parameters, "options.enable_road_boundary_linearization", enable_road_boundary_linearization_);
//   updateParam<bool>(parameters, "options.enable_two_way_road", enable_two_way_road_);

//   updateParam<double>(parameters, "road.width_right", road_width_right_);
//   updateParam<double>(parameters, "road.width_left", road_width_left_);

//   LMPCC_INFO_ALWAYS(logger_, "Parameters updated!");
//   auto result = rcl_interfaces::msg::SetParametersResult();
//   result.successful = true;
//   return result;
// }

// bool Contouring::ReadyForControl(SolverInterface *solver_interface, const RealTimeData &data)
// {
//   return ready_;
// }

// bool Contouring::ObjectiveReached(SolverInterface *solver_interface, const RealTimeData &data)
// {
//   return false;
//   // return goal_reached_ && !first_run_;
// }

// void Contouring::Update(SolverInterface *solver_interface, RealTimeData &data)
// {
//   PROFILE_AND_LOG(logger_, config_->debug_output_, "Contouring::Update");
//   first_run_ = false;

//   current_s_ = solver_interface->spline(0); //
//   UpdateClosestPoint(solver_.get(), current_s_);

//   // Check if the end of the path was reached
//   if (ReachedEnd())
//   {
//     if (!goal_reached_)
//     {
//       LMPCC_INFO(logger_, "Reached the end of the reference path!");
//       goal_reached_ = true;
//     }
//   }

//   if (EndOfCurrentSpline(current_s_) && !goal_reached_)
//     spline_index_++;

//   solver_interface->setInitialSpline(current_s_);
//   solver_interface->spline(0) = current_s_;

//   // Construct linearized constraints from the spline if enabled
//   if (enable_road_boundary_linearization_)
//   {
//     if (received_reference_path_.hasBounds())
//       ConstructRoadConstraintsFromData(solver_.get(), data.halfspaces_);
//     else
//       ConstructRoadConstraints(solver_.get(), data.halfspaces_); // Use the width of the road
//   }
// }

// void Contouring::OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name)
// {
//   if (data_name == "Waypoints")
//   {
//     LMPCC_INFO_ALWAYS(logger_, "Received Reference Path");

//     received_reference_path_ = data.reference_path_; // Save the new reference path data
//     OnWaypointsReceived();
//   }
//   else if (data_name == "State")
//   {
//     spline_index_ = InitializeClosestPoint(solver_interface);
//   }
// }

// void Contouring::ComputeDistanceVector(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &out)
// {
//   // Initialize the distance vector
//   out.clear();
//   out.resize(x.size());
//   out[0] = 0.;
//   for (size_t i = 1; i < x.size(); i++)
//   {
//     double dist = std::sqrt(std::pow(x[i] - x[i - 1], 2.) + std::pow(y[i] - y[i - 1], 2.));
//     out[i] = out[i - 1] + dist;
//   }
// }

// void Contouring::OnWaypointsReceived()
// {
//   LMPCC_INFO(logger_, "OnWaypointsReceived");

//   // Check validity
//   ROSTOOLS_ASSERT(received_reference_path_.isValid(), "Path is not valid!");

//   // If we received waypoints, assume that they are sufficiently smooth and directly use them as the cubic spline input
//   ProcessReceivedReferencePath(received_reference_path_);
// }

// void Contouring::Visualize()
// {
//   LMPCC_INFO(logger_, "Visualize");

//   if (config_->debug_output_)
//     PublishReferencePath();

//   PublishCurrentSplineIndex();
// }

// void Contouring::SetParameters(SolverInterface *solver_interface, const RealTimeData &data, int N_iter,
//                                int &param_idx)
// {

//   for (int i = 0; i < solver_interface->n_segments_; i++)
//   {

//     int index = std::min(reference_path_->NumberOfSegments(), (int)spline_index_ + i);

//     double ax, bx, cx, dx, ay, by, cy, dy;
//     reference_path_->GetParameters(index,
//                                    ax, bx, cx, dx,
//                                    ay, by, cy, dy);

//     // Spline x
//     solver_interface->setParameter(N_iter, param_idx, ax);
//     solver_interface->setParameter(N_iter, param_idx, bx);
//     solver_interface->setParameter(N_iter, param_idx, cx);
//     solver_interface->setParameter(N_iter, param_idx, dx);

//     // Spline y
//     solver_interface->setParameter(N_iter, param_idx, ay);
//     solver_interface->setParameter(N_iter, param_idx, by);
//     solver_interface->setParameter(N_iter, param_idx, cy);
//     solver_interface->setParameter(N_iter, param_idx, dy);

//     // Distance where this spline starts
//     solver_interface->setParameter(N_iter, param_idx, s_vector_[index]); // s1
//   }
// }

// double Contouring::FindClosestSRecursively(const Eigen::Vector2d &pose, double low, double high, int num_recursions)
// {
//   // Stop after x recursions
//   if (num_recursions > 20)
//   {
//     if (std::abs(high - low) > 1e-3)
//       LMPCC_WARN_ALWAYS(logger_, "FindClosestSRecursively did not find an accurate s (accuracy = "
//                                      << std::abs(high - low) << " | tolerance = 1e-3)");
//     return (low + high) / 2.;
//   }

//   // Computes the distance between point "s" on the spline and a vehicle position
//   auto dist_to_spline = [&](double s, const Eigen::Vector2d &pose)
//   {
//     return RosTools::dist(reference_path_->GetPoint(s), pose);
//     // return std::sqrt(std::pow(ref_path_x_(s) - pose(0), 2.) + std::pow(ref_path_y_(s) - pose(1), 2.));
//   };

//   // Compute a middle s value
//   double mid = (low + high) / 2.;

//   // Compute the distance to the spline for high/low
//   double value_low = dist_to_spline(low, pose);
//   double value_high = dist_to_spline(high, pose);

//   // Check the next closest value
//   if (value_low < value_high)
//     return FindClosestSRecursively(pose, low, mid, num_recursions + 1);
//   else
//     return FindClosestSRecursively(pose, mid, high, num_recursions + 1);
// }

// int Contouring::RecursiveClosestPointSearch(SolverInterface *solver_interface_ptr, unsigned int cur_traj_i,
//                                             double &s_guess, double window, int n_tries, int num_recursions)
// {
//   Eigen::Vector2d pose(solver_interface_ptr->State().x(), solver_interface_ptr->State().y());

//   s_guess = FindClosestSRecursively(pose, 0., s_vector_.back(), 0);

//   for (size_t i = 0; i < s_vector_.size() - 1; i++)
//   {
//     if (s_guess > s_vector_[i] && s_guess < s_vector_[i + 1])
//       return i; // Find the index to match the spline variable computed
//   }

//   return s_vector_.size() - 1;
// }

// void Contouring::UpdateClosestPoint(SolverInterface *solver_interface_ptr, double &s_guess, double window,
//                                     int n_tries)
// {
//   spline_index_ = RecursiveClosestPointSearch(solver_interface_ptr, spline_index_, s_guess, window, n_tries, 0);
// }

// double Contouring::InitializeClosestPoint(SolverInterface *solver_interface_ptr)
// {
//   Eigen::Vector2d current_pose(solver_interface_ptr->State().x(), solver_interface_ptr->State().y());

//   double smallest_dist = 9999999.0;
//   double current_dist;
//   int best_i = -1;
//   for (int i = 0; i < (int)s_vector_.size(); i++)
//   {
//     current_dist = RosTools::dist(current_pose, reference_path_->GetPoint(s_vector_[i]));

//     if (current_dist < smallest_dist)
//     {
//       smallest_dist = current_dist;
//       best_i = i;
//     }
//   }

//   if (best_i == -1)
//     LMPCC_ERROR(logger_, "Initial spline search failed: No point was found!");

//   // If it succeeded return our best index
//   if (best_i == -1)
//     return std::max(0, int(s_vector_.size() - 1));
//   else
//     return best_i;
// }

// bool Contouring::EndOfCurrentSpline(double index)
// {
//   if (spline_index_ + 1 < s_vector_.size()) // Checks if there are further splines
//     return index > s_vector_[spline_index_ + 1];
//   else
//     return false; // If there are no more splines, we cannot reach the end of this one
// }

// bool Contouring::ReachedEnd()
// {
//   return (size_t)(spline_index_ + 2) >= s_vector_.size(); // 1 for size, 2 for extra spline parts
// }

// void Contouring::ConstructRoadConstraintsFromData(SolverInterface *solver_interface,
//                                                   std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out)
// {
//   // For debugging
//   RosTools::ROSPointMarker &cube = ros_markers_linearboundaries->getNewPointMarker("Cube");
//   cube.setScale(0.5, 0.5, 0.5);
//   cube.setColor(1., 0., 0.);

//   geometry_msgs::msg::Point p;
//   p.z = 2.0;
//   for (size_t k = 0; k < solver_->FORCES_N; k++)
//   {
//     double cur_s = solver_->spline(k + 1); // + 1 because 0 is the initial

//     // Left side
//     Eigen::Vector2d A_left = left_bound_->GetOrthogonal(cur_s);
//     double b_left = A_left.transpose() * left_bound_->GetPoint(cur_s); // And lies on the boundary point
//     halfspaces_out[k][0] = RosTools::Halfspace(A_left, b_left);        // new_halfspace;

//     // Right side
//     Eigen::Vector2d A_right = right_bound_->GetOrthogonal(cur_s);
//     double b_right = A_right.transpose() * right_bound_->GetPoint(cur_s); // And lies on the boundary point
//     halfspaces_out[k][1] = RosTools::Halfspace(-A_right, -b_right);       // new_halfspace;

//     // For debugging
//     p.x = left_bound_->GetPoint(cur_s)(0);
//     p.y = left_bound_->GetPoint(cur_s)(1);
//     cube.addPointMarker(p);
//     p.x = right_bound_->GetPoint(cur_s)(0);
//     p.y = right_bound_->GetPoint(cur_s)(1);
//     cube.addPointMarker(p);
//   }

//   // Publish the constraints for debugging
//   PublishLinearRoadBoundaries(solver_interface, halfspaces_out);
// }

// void Contouring::ConstructRoadConstraints(SolverInterface *solver_interface,
//                                           std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out)
// {
//   LMPCC_INFO(logger_, "Reference Path: Constructing linear road constraints.");

//   RosTools::ROSPointMarker &cube = ros_markers_linearboundaries->getNewPointMarker("Cube");
//   cube.setScale(0.5, 0.5, 0.5);
//   cube.setColor(1., 0., 0.);

//   geometry_msgs::msg::Point p;
//   p.z = 2.0;

//   std::vector<double> x_path, y_path, dx_path, dy_path, lambdas;
//   x_path.resize(solver_->n_segments_);
//   y_path.resize(solver_->n_segments_);
//   dx_path.resize(solver_->n_segments_);
//   dy_path.resize(solver_->n_segments_);
//   lambdas.resize(solver_->n_segments_ - 1);

//   // OLD VERSION:
//   for (size_t k = 0; k < solver_interface->FORCES_N; k++)
//   {
//     double cur_s = solver_interface->spline(k + 1); // + 1 because 0 is the initial

//     // This is the final point and the normal vector of the path
//     Eigen::Vector2d path_point = reference_path_->GetPoint(cur_s); // Eigen::Vector2d(path_x, path_y);
//     Eigen::Vector2d dpath = reference_path_->GetOrthogonal(cur_s); // Eigen::Vector2d(-path_dy, path_dx);

//     // LEFT HALFSPACE
//     // dx, -dy NOTE 3* FOR DOUBLE LANE, 1* FOR SINGLE LANE
//     Eigen::Vector2d A = reference_path_->GetOrthogonal(cur_s); //(-path_dy, path_dx);
//     double width_times = enable_two_way_road_ ? 3.0 : 1.0;
//     // line is parallel to the spline
//     Eigen::Vector2d boundary_left =
//         path_point + dpath * (width_times * road_width_left_ -
//                               solver_interface->area_->DiscRadius()); // Incorporate the left road side
//     double b = A.transpose() * boundary_left;                         // And lies on the boundary point
//     halfspaces_out[k][0] = RosTools::Halfspace(A, b);                 // new_halfspace;

//     // RIGHT HALFSPACE
//     A = reference_path_->GetOrthogonal(cur_s); // Eigen::Vector2d(-path_dy, path_dx); // line is parallel to the spline

//     Eigen::Vector2d boundary_right =
//         path_point - dpath * (road_width_right_ - solver_interface->area_->DiscRadius());
//     b = A.transpose() * boundary_right; // And lies on the boundary point

//     halfspaces_out[k][1] = RosTools::Halfspace(-A, -b); // new_halfspace;
//     p.x = path_point(0);
//     p.y = path_point(1);
//     cube.addPointMarker(p);
//     p.x = boundary_right(0);
//     p.y = boundary_right(1);
//     cube.addPointMarker(p);
//     // p.x = boundary_left(0);
//     // p.y = boundary_left(1);
//     // cube.addPointMarker(p);
//   }

//   PublishLinearRoadBoundaries(solver_interface, halfspaces_out);
// }

// void Contouring::PublishReferencePath()
// {
//   RosTools::ROSLine &line = ros_markers_reference_path_->getNewLine();
//   line.setColorInt(10, 10);
//   line.setScale(0.1, 0.1);
//   line.setOrientation(0.0);

//   RosTools::ROSPointMarker &arrow = ros_markers_reference_arrows_->getNewPointMarker("ARROW");
//   arrow.setScale(1.5, 0.3, 0.3);

//   // Plot the part that we are tracking in the solver currently
//   RosTools::ROSLine &active_line = ros_markers_reference_path_->getNewLine();
//   active_line.setColorInt(8, 10, 0.5);
//   active_line.setScale(0.25, 0.25);
//   active_line.setOrientation(0.0);

//   geometry_msgs::msg::Point prev_point, p;
//   bool first = true;
//   for (double s = 0.; s < s_vector_.back(); s += 0.25)
//   {
//     auto cur_point = reference_path_->GetPoint(s);
//     auto cur_deriv = reference_path_->GetVelocity(s);
//     p.x = cur_point(0);
//     p.y = cur_point(1);

//     arrow.setOrientation(std::atan2(cur_deriv(1), cur_deriv(0)));
//     arrow.addPointMarker(p);

//     if (first)
//     {
//       first = false;
//     }
//     else
//     {
//       line.addLine(prev_point, p);

//       if (s > s_vector_[spline_index_] && s < s_vector_[spline_index_ + solver_->n_segments_])
//         active_line.addLine(prev_point, p);
//     }

//     prev_point = p;
//   }

//   ros_markers_reference_arrows_->publish();
//   ros_markers_reference_path_->publish();
// }

// void Contouring::PublishCurrentSplineIndex()
// {
//   // Use to debug spline init
//   RosTools::ROSPointMarker &cube = ros_markers_splineindex->getNewPointMarker("Cube");
//   cube.setScale(0.5, 0.5, 0.5);
//   cube.setOrientation(ref_theta_[spline_index_]);

//   geometry_msgs::msg::Point p;
//   p.z = 0.2;

//   auto &path = received_reference_path_.centerline;
//   if (path.x.size() > 0)
//   {
//     p.x = path.x[spline_index_];
//     p.y = path.y[spline_index_];
//     cube.addPointMarker(p);

//     cube.setColor(0, 0.8, 0);
//     p.x = path.x[spline_index_ + 1];
//     p.y = path.y[spline_index_ + 1];
//     cube.addPointMarker(p);

//     ros_markers_splineindex->publish();
//   }
// }

// void Contouring::PublishLinearRoadBoundaries(SolverInterface *solver_interface_ptr,
//                                              const std::vector<std::vector<RosTools::Halfspace>> &halfspaces_out)
// {
//   LMPCC_INFO(logger_, "Reference Path: Visualizing linear road constraints.");

//   for (size_t k = 0; k < solver_->FORCES_N; k++)
//     RosTools::DrawHalfspaces(*ros_markers_linearboundaries, halfspaces_out[k], k, 1.0e5);

//   ros_markers_linearboundaries->publish();
// }

// void Contouring::VisualizeRoad()
// {
//   RosTools::ROSLine &line = ros_markers_road_limits->getNewLine();
//   line.setColor(0.5, 0.0, 0.0);
//   line.setScale(0.1);
//   line.setOrientation(0.0);

//   geometry_msgs::msg::Point prev_l, cur_l, prev_r, cur_r, prev_m, cur_m;
//   cur_l.z = -0.5;
//   prev_l.z = -0.5;
//   cur_r.z = -0.5;
//   prev_r.z = -0.5;

//   for (size_t i = 0; i < s_vector_.size(); i++)
//   {
//     Eigen::Vector2d pose = reference_path_->GetPoint(s_vector_[i]);

//     // Vector orthogonal to the derivative of the spline
//     Eigen::Vector2d direction = reference_path_->GetOrthogonal(s_vector_[i]);
//     //(-ref_path_y_.deriv(1, s_vector_[i]), ref_path_x_.deriv(1, s_vector_[i]));

//     // Construct the road limits using the orthogonal vector
//     Eigen::Vector2d right = pose - direction * road_width_right_;
//     Eigen::Vector2d mid = pose + direction * road_width_left_;
//     Eigen::Vector2d left = pose + direction * road_width_left_ * 3.;

//     cur_l.x = left(0);
//     cur_l.y = left(1);

//     cur_m.x = mid(0);
//     cur_m.y = mid(1);

//     cur_r.x = right(0);
//     cur_r.y = right(1);

//     if (i > 0)
//     {
//       line.setColor(0.1, 0.1, 0.1); // 64./256., 224. / 256., 208 / 256.);
//       line.addLine(prev_l, cur_l);
//       line.addLine(prev_r, cur_r);

//       if (i % 2 == 1)
//       {
//         line.setColor(249. / 256., 215. / 256., 28 / 256.);
//         line.addLine(prev_m, cur_m);
//       }
//     }

//     prev_l = cur_l;
//     prev_m = cur_m;
//     prev_r = cur_r;
//   }

//   ros_markers_road_limits->publish();
// }
// * /