#include "mpc_planner_modules/contouring_constraints.h"

#include <mpc_planner_parameters.h>

#include <mpc_planner_util/parameters.h>

#include <ros_tools/visuals.h>
#include <ros_tools/spline.h>
#include <ros_tools/math.h>

namespace MPCPlanner
{
  ContouringConstraints::ContouringConstraints(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "contouring_constraints")
  {
    LOG_INITIALIZE("Contouring Constraints");
    LOG_INITIALIZED();

    _num_segments = CONFIG["contouring"]["num_segments"].as<int>();
  }

  void ContouringConstraints::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;
    (void)data;

    if (module_data.path_width_left == nullptr && _width_left != nullptr)
      module_data.path_width_left = _width_left;

    if (module_data.path_width_right == nullptr && _width_right != nullptr)
      module_data.path_width_right = _width_right;
  }

  void ContouringConstraints::onDataReceived(RealTimeData &data, std::string &&data_name)
  {
    if (data_name == "reference_path")
    {
      LOG_MARK("Reference Path Received");

      if (!data.left_bound.empty() && !data.right_bound.empty())
      {
        LOG_MARK("Received Road Boundaries");

        std::vector<double> widths_left, widths_right;
        widths_right.resize(data.right_bound.x.size());
        widths_left.resize(data.left_bound.x.size());

        for (size_t i = 0; i < widths_left.size(); i++)
        {
          Eigen::Vector2d center(data.reference_path.x[i], data.reference_path.y[i]);
          Eigen::Vector2d left(data.left_bound.x[i], data.left_bound.y[i]);
          Eigen::Vector2d right(data.right_bound.x[i], data.right_bound.y[i]);
          widths_left[i] = RosTools::distance(center, left);
          widths_right[i] = RosTools::distance(center, right);
        }

        std::vector<double> s_vec;

        _width_left = std::make_shared<tk::spline>();
        _width_left->set_points(data.reference_path.s, widths_left);
        // _width_left->set_points(spline.getTVector(), widths_left);

        _width_right = std::make_shared<tk::spline>();
        _width_right->set_points(data.reference_path.s, widths_right);

        // _width_right->set_points(spline.getTVector(), widths_right);
        // CONFIG["road"]["width"] = _width_right->operator()(0.) - _width_left->operator()(0.);
      }
    }
  }

  void ContouringConstraints::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)module_data;

    if (k == 1)
      LOG_MARK("ContouringConstraints::setParameters");

    for (int i = 0; i < _num_segments; i++)
    {
      int index = module_data.current_path_segment + i;

      // Boundaries
      double ra, rb, rc, rd;
      double la, lb, lc, ld;

      if (index < _width_right->m_x_.size() - 1)
      {

        _width_right->getParameters(index, ra, rb, rc, rd);
        _width_left->getParameters(index, la, lb, lc, ld);
      }
      else
      {
        _width_right->getParameters(_width_right->m_x_.size() - 1, ra, rb, rc, rd);
        _width_left->getParameters(_width_left->m_x_.size() - 1, la, lb, lc, ld);

        ra = 0.;
        rb = 0.;
        rc = 0.;
        la = 0.;
        lb = 0.;
        lc = 0.;
      }

      // Boundary
      setSolverParameterWidthRightA(k, _solver->_params, ra, i);
      setSolverParameterWidthRightB(k, _solver->_params, rb, i);
      setSolverParameterWidthRightC(k, _solver->_params, rc, i);
      setSolverParameterWidthRightD(k, _solver->_params, rd, i);

      setSolverParameterWidthLeftA(k, _solver->_params, la, i);
      setSolverParameterWidthLeftB(k, _solver->_params, lb, i);
      setSolverParameterWidthLeftC(k, _solver->_params, lc, i);
      setSolverParameterWidthLeftD(k, _solver->_params, ld, i);
    }

    if (k == 1)
      LOG_MARK("ContouringConstraints::setParameters Done");
  }

  bool ContouringConstraints::isDataReady(const RealTimeData &data, std::string &missing_data)
  {
    if (data.left_bound.empty() || data.right_bound.empty())
    {
      missing_data += "Road Bounds ";
      return false;
    }

    return true;
  }

  void ContouringConstraints::visualize(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)data;
    (void)module_data;

    if (!CONFIG["debug_visuals"].as<bool>())
      return;

    LOG_MARK("ContouringConstraints::Visualize");

    if (_width_right == nullptr || _width_left == nullptr || module_data.path == nullptr)
      return;

    auto &line_publisher = VISUALS.getPublisher(_name + "/road_boundary");
    auto &line = line_publisher.getNewLine();
    line.setScale(0.1);
    line.setColorInt(0);

    Eigen::Vector2d prev_right, prev_left;

    for (double cur_s = 0.; cur_s < _width_right->m_x_.back(); cur_s += 0.5)
    {
      double right = _width_right->operator()(cur_s);
      double left = _width_left->operator()(cur_s);

      Eigen::Vector2d path_point = module_data.path->getPoint(cur_s);
      Eigen::Vector2d dpath = module_data.path->getOrthogonal(cur_s);

      if (cur_s > 0)
      {
        line.addLine(prev_left, path_point - dpath * left);
        line.addLine(prev_right, path_point + dpath * right);
      }

      prev_left = path_point - dpath * left;
      prev_right = path_point + dpath * right;
    }
    line_publisher.publish();

    // if (!CONFIG["debug_visuals"].as<bool>())
    // return;

    auto &publisher = VISUALS.getPublisher(_name + "/road_boundary_points");
    auto &points = publisher.getNewPointMarker("CUBE");
    auto &contour_line = publisher.getNewLine();
    contour_line.setScale(0.15);
    points.setScale(0.15, 0.15, 0.15);

    for (int k = 1; k < _solver->N; k++)
    {

      double cur_s = _solver->getOutput(k, "spline");
      Eigen::Vector2d path_point = module_data.path->getPoint(cur_s);

      points.setColorInt(5, 10);
      points.addPointMarker(path_point);

      Eigen::Vector2d dpath = module_data.path->getOrthogonal(cur_s);

      // line is parallel to the spline
      Eigen::Vector2d boundary_left = path_point - dpath * (_width_left->operator()(cur_s));
      Eigen::Vector2d boundary_right = path_point + dpath * (_width_right->operator()(cur_s));

      // Visualize the contouring error
      double w_cur = CONFIG["robot"]["width"].as<double>() / 2.;
      Eigen::Vector2d pos(_solver->getOutput(k, "x"), _solver->getOutput(k, "y"));

      points.setColor(0., 0., 0.);
      points.addPointMarker(pos, 0.2); // Planned positions and black dots

      double contour_error = dpath.transpose() * (pos - path_point);
      double w_right = _width_right->operator()(cur_s);
      double w_left = _width_left->operator()(cur_s);

      contour_line.setColor(1., 0., 0.); // Red contour error
      contour_line.addLine(path_point, path_point + dpath * contour_error, 0.1);

      contour_line.setColor(0., 1., 0.); // Width left and right in green and blue
      contour_line.addLine(path_point, path_point + dpath * (-w_left + w_cur));

      contour_line.setColor(0., 0., 1.);
      contour_line.addLine(path_point, path_point + dpath * (w_right - w_cur));

      points.setColor(0., 0., 0.);
      points.addPointMarker(boundary_left);
      points.addPointMarker(boundary_right);
    }
    publisher.publish();
  }

} // namespace MPCPlanner
