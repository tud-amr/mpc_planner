#include "mpc_planner_modules/contouring_constraints.h"

#include <mpc_planner_generated.h>

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
    (void)module_data;
  }

  void ContouringConstraints::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)module_data;

    if (k == 1)
      LOG_MARK("ContouringConstraints::setParameters");

    for (int i = 0; i < _num_segments; i++)
    {
      int index = data.reference_path.current_segment + i;

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
      setForcesParameterWidthRightA(k, _solver->_params, ra, i);
      setForcesParameterWidthRightB(k, _solver->_params, rb, i);
      setForcesParameterWidthRightC(k, _solver->_params, rc, i);
      setForcesParameterWidthRightD(k, _solver->_params, rd, i);

      setForcesParameterWidthLeftA(k, _solver->_params, la, i);
      setForcesParameterWidthLeftB(k, _solver->_params, lb, i);
      setForcesParameterWidthLeftC(k, _solver->_params, lc, i);
      setForcesParameterWidthLeftD(k, _solver->_params, ld, i);
    }

    if (k == 1)
      LOG_MARK("ContouringConstraints::setParameters Done");
  }

  void ContouringConstraints::onDataReceived(RealTimeData &data, std::string &&data_name)
  {
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

      _width_left = std::make_unique<tk::spline>();
      _width_left->set_points(data.reference_path.s, widths_left);

      _width_right = std::make_unique<tk::spline>();
      _width_right->set_points(data.reference_path.s, widths_right);
      // CONFIG["road"]["width"] = _width_right->operator()(0.) - _width_left->operator()(0.);

      // Add bounds
      // _bound_left = std::make_unique<RosTools::Spline2D>(
      //     data.left_bound.x,
      //     data.left_bound.y,
      //     _spline->getTVector());
      // _bound_right = std::make_unique<RosTools::Spline2D>(
      //     data.right_bound.x,
      //     data.right_bound.y,
      //     _spline->getTVector());

      // // Update the road width
      // CONFIG["road"]["width"] = RosTools::distance(_bound_left->getPoint(0), _bound_right->getPoint(0));
      // if (_two_way_road)
      // CONFIG["road"]["width"] = CONFIG["road"]["width"].as<double>() / 2.;
    }
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
  }

} // namespace MPCPlanner
