#include "mpc_planner_modules/contouring.h"

#include <mpc_planner_generated.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/data_visualization.h>

#include <ros_tools/visuals.h>
#include <ros_tools/math.h>
#include <ros_tools/profiling.h>

#include <algorithm>

namespace MPCPlanner
{
  Contouring::Contouring(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "contouring")
  {
    LOG_INITIALIZE("Contouring");
    _n_segments = CONFIG["contouring"]["num_segments"].as<int>();
    _add_road_constraints = CONFIG["contouring"]["add_road_constraints"].as<bool>();
    _two_way_road = CONFIG["road"]["two_way"].as<bool>();

    LOG_INITIALIZED();
  }

  void Contouring::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)data;
    PROFILE_SCOPE("Contouring Update");

    LOG_DEBUG("contouring::update()");

    // Update the closest point
    double closest_s;
    _spline->findClosestPoint(state.getPos(), _closest_segment, closest_s);

    state.set("spline", closest_s); // We need to initialize the spline state here

    module_data.current_path_segment = _closest_segment;

    if (_add_road_constraints)
      constructRoadConstraints(data, module_data);
  }

  void Contouring::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)data;
    (void)module_data;

    // Retrieve weights once
    static double contouring_weight, lag_weight, preview_weight;
    if (k == 0)
    {
      contouring_weight = CONFIG["weights"]["contour"].as<double>();
      lag_weight = CONFIG["weights"]["lag"].as<double>();
      preview_weight = CONFIG["weights"]["preview"].as<double>();
    }

    {
      setForcesParameterContour(k, _solver->_params, contouring_weight);
      setForcesParameterLag(k, _solver->_params, lag_weight);

      if (preview_weight > 0.)
        _solver->setParameter(k, "preview", preview_weight);
    }
    for (int i = 0; i < _n_segments; i++)
    {
      int index = _closest_segment + i;
      double ax, bx, cx, dx;
      double ay, by, cy, dy;

      // Boundaries
      double ra, rb, rc, rd;
      double la, lb, lc, ld;
      double start;

      if (index < _spline->numSegments())
      {
        _spline->getParameters(index,
                               ax, bx, cx, dx,
                               ay, by, cy, dy);

        _width_right->getParameters(index,
                                    ra, rb, rc, rd);
        _width_left->getParameters(index,
                                   la, lb, lc, ld);

        start = _spline->getSegmentStart(index);
      }
      else
      {
        // If we are beyond the spline, we should use the last spline
        _spline->getParameters(_spline->numSegments() - 1,
                               ax, bx, cx, dx,
                               ay, by, cy, dy);

        _width_right->getParameters(_spline->numSegments() - 1,
                                    ra, rb, rc, rd);
        _width_left->getParameters(_spline->numSegments() - 1,
                                   la, lb, lc, ld);

        start = _spline->getSegmentStart(_spline->numSegments() - 1);

        // We should use very small splines at the end location
        // x = d_x
        // y = d_y
        ax = 0.;
        bx = 0.;
        cx = 0.;
        ay = 0.;
        by = 0.;
        cy = 0.;

        ra = 0.;
        rb = 0.;
        rc = 0.;
        la = 0.;
        lb = 0.;
        lc = 0.;
        start = _spline->parameterLength();
      }

      /** @note: We use the fast loading interface here as we need to load many parameters */
      setForcesParameterSplineXA(k, _solver->_params, ax, i);
      setForcesParameterSplineXB(k, _solver->_params, bx, i);
      setForcesParameterSplineXC(k, _solver->_params, cx, i);
      setForcesParameterSplineXD(k, _solver->_params, dx, i);

      setForcesParameterSplineYA(k, _solver->_params, ay, i);
      setForcesParameterSplineYB(k, _solver->_params, by, i);
      setForcesParameterSplineYC(k, _solver->_params, cy, i);
      setForcesParameterSplineYD(k, _solver->_params, dy, i);

      // Boundary
      // setForcesParameterBoundRightA(k, _solver->_params, ra, i);
      // setForcesParameterBoundRightB(k, _solver->_params, rb, i);
      // setForcesParameterBoundRightC(k, _solver->_params, rc, i);
      // setForcesParameterBoundRightD(k, _solver->_params, rd, i);

      // setForcesParameterBoundLeftA(k, _solver->_params, la, i);
      // setForcesParameterBoundLeftB(k, _solver->_params, lb, i);
      // setForcesParameterBoundLeftC(k, _solver->_params, lc, i);
      // setForcesParameterBoundLeftD(k, _solver->_params, ld, i);

      // Distance where this spline starts
      setForcesParameterSplineStart(k, _solver->_params, start, i);
    }
  }

  void Contouring::onDataReceived(RealTimeData &data, std::string &&data_name)
  {
    if (data_name == "reference_path")
    {
      LOG_MARK("Received Reference Path");

      // Construct a spline from the given points
      _spline = std::make_unique<RosTools::Spline2D>(data.reference_path.x, data.reference_path.y);

      if (!data.left_bound.empty() && !data.right_bound.empty())
      {
        std::vector<double> widths_left, widths_right;
        widths_right.resize(data.right_bound.x.size());
        widths_left.resize(data.left_bound.x.size());

        for (size_t i = 0; i < widths_left.size(); i++)
        {
          Eigen::Vector2d center(data.reference_path.x[i], data.reference_path.y[i]);
          Eigen::Vector2d left(data.left_bound.x[i], data.left_bound.y[i]);
          Eigen::Vector2d right(data.right_bound.x[i], data.right_bound.y[i]);
          widths_left[i] = -RosTools::distance(center, left); // Minus because contouring erros is negative on this side
          widths_right[i] = RosTools::distance(center, right);
        }

        _width_left = std::make_unique<tk::spline>();
        _width_left->set_points(_spline->getTVector(), widths_left);

        _width_right = std::make_unique<tk::spline>();
        _width_right->set_points(_spline->getTVector(), widths_right);
        // CONFIG["road"]["width"] = _width_right->operator()(0.) - _width_left->operator()(0.);

        // Add bounds
        _bound_left = std::make_unique<RosTools::Spline2D>(
            data.left_bound.x,
            data.left_bound.y,
            _spline->getTVector());
        _bound_right = std::make_unique<RosTools::Spline2D>(
            data.right_bound.x,
            data.right_bound.y,
            _spline->getTVector());

        // Update the road width
        CONFIG["road"]["width"] = RosTools::distance(_bound_left->getPoint(0), _bound_right->getPoint(0));
        // if (_two_way_road)
        // CONFIG["road"]["width"] = CONFIG["road"]["width"].as<double>() / 2.;
      }

      _closest_segment = -1;
    }
  }

  bool Contouring::isDataReady(const RealTimeData &data, std::string &missing_data)
  {
    if (data.reference_path.x.empty())
      missing_data += "Reference Path ";

    return !data.reference_path.x.empty();
  }

  bool Contouring::isObjectiveReached(const State &state, const RealTimeData &data)
  {
    (void)data;

    if (!_spline)
      return false;

    // Check if we reached the end of the spline
    return RosTools::distance(state.getPos(), _spline->getPoint(_spline->parameterLength())) < 0.1;

    // int index = _closest_segment + _n_segments - 1;
    // return index >= _spline->numSegments();
  }

  void Contouring::constructRoadConstraints(const RealTimeData &data, ModuleData &module_data)
  {
    LOG_MARK("Constructing road constraints.");

    if (data.left_bound.empty() || data.right_bound.empty())
      constructRoadConstraintsFromCenterline(data, module_data);
    else
      constructRoadConstraintsFromBounds(data, module_data);
  }

  void Contouring::constructRoadConstraintsFromCenterline(const RealTimeData &data, ModuleData &module_data)
  {
    /** @brief If bounds are not supplied construct road constraints based on a set width*/
    if (module_data.static_obstacles.empty())
    {
      module_data.static_obstacles.resize(_solver->N);
      for (size_t k = 0; k < module_data.static_obstacles.size(); k++)
        module_data.static_obstacles[k].reserve(2);
    }

    // OLD VERSION:
    bool two_way = _two_way_road;
    double road_width_half = CONFIG["road"]["width"].as<double>() / 2.;
    for (int k = 1; k < _solver->N; k++)
    {
      module_data.static_obstacles[k].clear();

      double cur_s = _solver->getEgoPrediction(k, "spline");

      // This is the final point and the normal vector of the path
      Eigen::Vector2d path_point = _spline->getPoint(cur_s);
      Eigen::Vector2d dpath = _spline->getOrthogonal(cur_s);

      // LEFT HALFSPACE
      Eigen::Vector2d A = _spline->getOrthogonal(cur_s);
      double width_times = two_way ? 3.0 : 1.0; // 3w for double lane

      // line is parallel to the spline
      Eigen::Vector2d boundary_left =
          path_point + dpath * (width_times * road_width_half - data.robot_area[0].radius);

      double b = A.transpose() * boundary_left;

      module_data.static_obstacles[k].emplace_back(A, b);

      // RIGHT HALFSPACE
      A = _spline->getOrthogonal(cur_s); // Eigen::Vector2d(-path_dy, path_dx); // line is parallel to the spline

      Eigen::Vector2d boundary_right =
          path_point - dpath * (road_width_half - data.robot_area[0].radius);
      b = A.transpose() * boundary_right; // And lies on the boundary point

      module_data.static_obstacles[k].emplace_back(-A, -b);
    }
  }

  void Contouring::constructRoadConstraintsFromBounds(const RealTimeData &data, ModuleData &module_data)
  {
    /** @todo */
    if (module_data.static_obstacles.empty())
    {
      module_data.static_obstacles.resize(_solver->N);
      for (size_t k = 0; k < module_data.static_obstacles.size(); k++)
        module_data.static_obstacles[k].reserve(2);
    }

    for (int k = 1; k < _solver->N; k++)
    {
      module_data.static_obstacles[k].clear();
      double cur_s = _solver->getEgoPrediction(k, "spline");

      // Left
      Eigen::Vector2d Al = _bound_left->getOrthogonal(cur_s);
      double bl = Al.transpose() * (_bound_left->getPoint(cur_s) + Al * data.robot_area[0].radius);
      module_data.static_obstacles[k].emplace_back(-Al, -bl);

      // RIGHT HALFSPACE
      Eigen::Vector2d Ar = _bound_right->getOrthogonal(cur_s);
      double br = Ar.transpose() * (_bound_right->getPoint(cur_s) - Ar * data.robot_area[0].radius);
      module_data.static_obstacles[k].emplace_back(Ar, br);
    }
  }

  void Contouring::visualize(const RealTimeData &data, const ModuleData &module_data)
  {
    if (_spline.get() == nullptr)
      return;

    PROFILE_SCOPE("Contouring::Visualize");

    visualizeReferencePath(data, module_data);

    visualizeRoadConstraints(data, module_data);

    if (CONFIG["debug_visuals"].as<bool>())
    {
      visualizeCurrentSegment(data, module_data);
      visualizeDebugRoadBoundary(data, module_data);
      visualizeDebugGluedSplines(data, module_data);
      visualizeAllSplineIndices(data, module_data);
    }
  }

  void Contouring::visualizeCurrentSegment(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)data;
    (void)module_data;

    // Visualize the current points
    auto &publisher_current = VISUALS.getPublisher(_name + "/current");
    auto &cur_point = publisher_current.getNewPointMarker("CUBE");
    cur_point.setColorInt(10);
    cur_point.setScale(0.3, 0.3, 0.3);
    cur_point.addPointMarker(_spline->getPoint(_spline->getSegmentStart(_closest_segment)), 0.0);
    publisher_current.publish();
  }

  // Move to data visualization
  void Contouring::visualizeReferencePath(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)module_data;
    visualizePathPoints(data.reference_path, _name + "/points", true);
    visualizeSpline(*_spline, _name + "/path", true);

    if (!data.left_bound.empty())
    {

      visualizePathPoints(data.left_bound, _name + "/boundary_points", false);
      visualizePathPoints(data.right_bound, _name + "/boundary_points", true);
    }
    if (_bound_left != nullptr)
    {
      visualizeSpline(*_bound_left, _name + "/boundary_path", false);
      visualizeSpline(*_bound_right, _name + "/boundary_path", true);
    }
  }

  void Contouring::visualizeRoadConstraints(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)data;
    if (module_data.static_obstacles.empty() || (!_add_road_constraints))
      return;

    for (int k = 1; k < _solver->N; k++)
    {
      for (size_t h = 0; h < module_data.static_obstacles[k].size(); h++)
      {
        visualizeLinearConstraint(module_data.static_obstacles[k][h],
                                  k, _solver->N,
                                  _name + "/road_boundary_constraints",
                                  k == _solver->N - 1 && h == module_data.static_obstacles[k].size() - 1,
                                  0.5, 0.1);
      }
    }
  }

  void Contouring::visualizeDebugRoadBoundary(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)module_data;
    auto &publisher = VISUALS.getPublisher(_name + "/road_boundary_points");
    auto &points = publisher.getNewPointMarker("CUBE");
    points.setScale(0.15, 0.15, 0.15);

    // OLD VERSION:
    bool two_way = _two_way_road;
    double road_width_half = CONFIG["road"]["width"].as<double>() / 2.;
    for (int k = 1; k < _solver->N; k++)
    {

      double cur_s = _solver->getEgoPrediction(k, "spline");
      Eigen::Vector2d path_point = _spline->getPoint(cur_s);

      points.setColorInt(5, 10);
      points.addPointMarker(path_point);

      Eigen::Vector2d dpath = _spline->getOrthogonal(cur_s);

      double width_times = two_way ? 3.0 : 1.0; // 3w for double lane

      // line is parallel to the spline
      Eigen::Vector2d boundary_left =
          path_point + dpath * (width_times * road_width_half - data.robot_area[0].radius);

      Eigen::Vector2d boundary_right =
          path_point - dpath * (road_width_half - data.robot_area[0].radius);

      points.setColor(0., 0., 0.);
      points.addPointMarker(boundary_left);
      points.addPointMarker(boundary_right);
    }
    publisher.publish();
  }

  void Contouring::visualizeDebugGluedSplines(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)data;
    (void)module_data;

    // Plot how the optimization joins the splines together to debug its internal contouring error computation
    auto &publisher = VISUALS.getPublisher(_name + "/glued_spline_points");
    auto &points = publisher.getNewPointMarker("CUBE");
    points.setScale(0.15, 0.15, 0.15);

    for (int k = 1; k < _solver->N; k++)
    {
      std::vector<double> lambdas;
      std::vector<double> path_x, path_y;

      for (int i = 0; i < _n_segments; i++)
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

          start = _spline->getSegmentStart(index);
        }
        else
        {
          // If we are beyond the spline, we should use the last spline
          _spline->getParameters(_spline->numSegments() - 1,
                                 ax, bx, cx, dx,
                                 ay, by, cy, dy);

          start = _spline->getSegmentStart(_spline->numSegments() - 1);

          // We should use very small splines at the end location
          // x = d_x
          // y = d_y
          ax = 0.;
          bx = 0.;
          cx = 0.;
          ay = 0.;
          by = 0.;
          cy = 0.;
          start = _spline->parameterLength();
        }

        double s = _solver->getEgoPrediction(k, "spline") - start;
        path_x.push_back(ax * s * s * s + bx * s * s + cx * s + dx);
        path_y.push_back(ay * s * s * s + by * s * s + cy * s + dy);

        // No lambda for the first segment (it is not glued to anything prior)
        if (i > 0)
        {
          lambdas.push_back(1.0 / (1.0 + std::exp((s + 0.02) / 0.1))); // Sigmoid
        }
      }

      double cur_path_x = path_x.back();
      double cur_path_y = path_y.back();
      for (int p = path_x.size() - 1; p > 0; p--)
      {
        // Glue with the previous path
        cur_path_x = lambdas[p - 1] * path_x[p - 1] + (1.0 - lambdas[p - 1]) * cur_path_x;
        cur_path_y = lambdas[p - 1] * path_y[p - 1] + (1.0 - lambdas[p - 1]) * cur_path_y;
      }

      points.addPointMarker(Eigen::Vector2d(cur_path_x, cur_path_y));
    }
    publisher.publish();
  }

  void Contouring::visualizeAllSplineIndices(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)data;
    (void)module_data;

    // Plot how the optimization joins the splines together to debug its internal contouring error computation
    auto &publisher = VISUALS.getPublisher(_name + "/spline_variables");
    auto &points = publisher.getNewPointMarker("CUBE");
    points.setScale(0.15, 0.15, 0.15);

    for (int k = 0; k < _solver->N; k++)
    {
      double cur_s = _solver->getEgoPrediction(k, "spline");
      Eigen::Vector2d path_point = _spline->getPoint(cur_s);
      points.addPointMarker(path_point);
    }

    publisher.publish();
  }

  void Contouring::reset()
  {
    _spline.reset(nullptr);
    _closest_segment = 0;
  }

} // namespace MPCPlanner