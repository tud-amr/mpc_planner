#include "mpc_planner_modules/contouring.h"

#include <mpc_planner_generated.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/data_visualization.h>

#include <ros_tools/visuals.h>
#include <ros_tools/profiling.h>

#include <algorithm>

namespace MPCPlanner
{
  Contouring::Contouring(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "contouring")
  {
    LOG_INITIALIZE("Contouring");
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

    constructRoadConstraints(data, module_data);
  }

  void Contouring::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)data;
    (void)module_data;

    _solver->setParameter(k, "contour", CONFIG["weights"]["contour"].as<double>());
    _solver->setParameter(k, "lag", CONFIG["weights"]["lag"].as<double>());

    // Add condition
    if (_solver->hasParameter("preview"))
      _solver->setParameter(k, "preview", CONFIG["weights"]["preview"].as<double>());

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

        start = _spline->getSegmentStart(index);
      }
      else
      {
        LOG_WARN_THROTTLE(3000, "Beyond the spline");
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

      /** @note: We use the fast loading interface here as we need to load many parameters */
      setForcesParameterSplineAx(k, _solver->_params, ax, i);
      setForcesParameterSplineBx(k, _solver->_params, bx, i);
      setForcesParameterSplineCx(k, _solver->_params, cx, i);
      setForcesParameterSplineDx(k, _solver->_params, dx, i);

      setForcesParameterSplineAy(k, _solver->_params, ay, i);
      setForcesParameterSplineBy(k, _solver->_params, by, i);
      setForcesParameterSplineCy(k, _solver->_params, cy, i);
      setForcesParameterSplineDy(k, _solver->_params, dy, i);

      // Distance where this spline starts
      setForcesParameterSplineStart(k, _solver->_params, start, i);
    }
  }

  void Contouring::onDataReceived(RealTimeData &data, std::string &&data_name)
  {
    if (data_name == "reference_path")
    {
      LOG_INFO("Received Reference Path");

      // Construct a spline from the given points
      _spline = std::make_unique<RosTools::Spline2D>(data.reference_path.x, data.reference_path.y);

      if (!data.left_bound.empty() && !data.right_bound.empty())
      {
        // Add bounds
        _bound_left = std::make_unique<RosTools::Spline2D>(
            data.left_bound.x,
            data.left_bound.y,
            _spline->getTVector());
        _bound_right = std::make_unique<RosTools::Spline2D>(
            data.right_bound.x,
            data.right_bound.y,
            _spline->getTVector());
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

  bool Contouring::isObjectiveReached(const RealTimeData &data)
  {
    (void)data;

    if (!_spline)
      return false;

    int index = _closest_segment + CONFIG["contouring"]["num_segments"].as<int>() - 1;
    return index >= _spline->numSegments();
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
    module_data.static_obstacles.resize(_solver->N);

    // OLD VERSION:
    bool two_way = CONFIG["road"]["two_way"].as<bool>();
    double road_width_half = CONFIG["road"]["width"].as<double>() / 2.;
    for (int k = 1; k < _solver->N; k++)
    {
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
    module_data.static_obstacles.resize(_solver->N);

    for (int k = 1; k < _solver->N; k++)
    {
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

    visualizeReferencePath(data, module_data);
    visualizeCurrentSegment(data, module_data);

    visualizeRoadConstraints(data, module_data);

    visualizeDebugRoadBoundary(data, module_data);
    visualizeDebugGluedSplines(data, module_data);
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
    visualizePathPoints(data.left_bound, _name + "/boundary_points", false);
    visualizePathPoints(data.right_bound, _name + "/boundary_points", true);
    // // Visualize the points
    // auto &publisher_points = VISUALS.getPublisher(_name + "/points");
    // auto &point = publisher_points.getNewPointMarker("CYLINDER");
    // point.setColor(0., 0., 0.);
    // point.setScale(0.15, 0.15, 0.05);

    // for (size_t p = 0; p < data.reference_path.x.size(); p++)
    //   point.addPointMarker(Eigen::Vector3d(data.reference_path.x[p], data.reference_path.y[p], 0.1));
    // publisher_points.publish();

    visualizeSpline(*_spline, _name + "/path", true);

    if (_bound_left != nullptr)
    {
      visualizeSpline(*_bound_left, _name + "/boundary_path", false);
      visualizeSpline(*_bound_right, _name + "/boundary_path", true);
    }
    // // Visualize the path
    // auto &publisher_path = VISUALS.getPublisher(_name + "/path");
    // auto &line = publisher_path.getNewLine();
    // line.setColorInt(5);
    // line.setScale(0.1);

    // Eigen::Vector2d p;
    // for (double s = 0.; s < _spline->parameterLength(); s += 1.)
    // {
    //   if (s > 0.)
    //     line.addLine(p, _spline->getPoint(s));

    //   p = _spline->getPoint(s);
    // }
    // line.addLine(p, _spline->getPoint(_spline->parameterLength())); // Connect to the end

    // publisher_path.publish();
  }

  void Contouring::visualizeRoadConstraints(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)data;
    if (module_data.static_obstacles.empty())
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
    LOG_MARK("Constructing road constraints.");
    auto &publisher = VISUALS.getPublisher(_name + "/road_boundary_points");
    auto &points = publisher.getNewPointMarker("CUBE");
    points.setScale(0.15, 0.15, 0.15);

    // OLD VERSION:
    bool two_way = CONFIG["road"]["two_way"].as<bool>();
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

  void Contouring::reset()
  {
    _spline.reset(nullptr);
    _closest_segment = 0;
  }

} // namespace MPCPlanner