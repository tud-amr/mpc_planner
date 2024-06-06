
#include <mpc_planner_modules/path_reference_velocity.h>

#include <mpc_planner_parameters.h>
#include <mpc_planner_util/parameters.h>

#include <ros_tools/visuals.h>
#include <ros_tools/spline.h>

namespace MPCPlanner
{

  PathReferenceVelocity::PathReferenceVelocity(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "path_reference_velocity")
  {
    _n_segments = CONFIG["contouring"]["num_segments"].as<int>();
  }

  void PathReferenceVelocity::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;
    (void)data;

    if (module_data.path_velocity == nullptr && _velocity_spline != nullptr)
      module_data.path_velocity = _velocity_spline;
  }

  void PathReferenceVelocity::onDataReceived(RealTimeData &data, std::string &&data_name)
  {
    if (data_name == "reference_path")
    {
      LOG_MARK("Received Reference Path");

      if (data.reference_path.hasVelocity())
      {
        _velocity_spline = std::make_shared<tk::spline>();
        _velocity_spline->set_points(data.reference_path.s, data.reference_path.v);
      }
    }
  }

  void PathReferenceVelocity::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)module_data;
    (void)data;

    static double reference_velocity; //, velocity_weight;

    // Retrieve once
    if (k == 0)
    {
      // velocity_weight = CONFIG["weights"]["velocity"].as<double>();
      reference_velocity = CONFIG["weights"]["reference_velocity"].as<double>();
    }

    // Set the parameters for velocity tracking
    // setSolverParameterVelocity(k, _solver->_params, velocity_weight);

    if (data.reference_path.hasVelocity()) // Use a spline-based velocity reference
    {
      LOG_MARK("Using spline-based reference velocity");
      for (int i = 0; i < _n_segments; i++)
      {
        int index = module_data.current_path_segment + i;
        double a, b, c, d;

        if (index < (int)_velocity_spline->m_x_.size() - 1)
        {
          _velocity_spline->getParameters(index, a, b, c, d);
        }
        else
        {
          // Brake at the end
          a = 0.;
          b = 0.;
          c = 0.;
          d = 0.;
        }

        setSolverParameterSplineVA(k, _solver->_params, a, i);
        setSolverParameterSplineVB(k, _solver->_params, b, i);
        setSolverParameterSplineVC(k, _solver->_params, c, i);
        setSolverParameterSplineVD(k, _solver->_params, d, i);
      }
    }
    else // Use a constant velocity reference
    {
      for (int i = 0; i < _n_segments; i++)
      {
        setSolverParameterSplineVA(k, _solver->_params, 0., i);
        setSolverParameterSplineVB(k, _solver->_params, 0., i);
        setSolverParameterSplineVC(k, _solver->_params, 0., i);
        setSolverParameterSplineVD(k, _solver->_params, reference_velocity, i); // v = d
      }
    }
  }

  void PathReferenceVelocity::visualize(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)module_data;
    if (data.reference_path.empty())
      return;

    if (!CONFIG["debug_visuals"].as<bool>())
      return;

    LOG_MARK("PathReferenceVelocity::Visualize");

    // Only for debugging
    auto &publisher = VISUALS.getPublisher("path_velocity");
    auto &line = publisher.getNewLine();

    line.setScale(0.25, 0.25, 0.1);
    auto spline_xy = std::make_unique<RosTools::Spline2D>(data.reference_path.x, data.reference_path.y, data.reference_path.s);

    Eigen::Vector2d prev;
    double prev_v = 0.;
    for (double s = 0.; s < _velocity_spline->m_x_.back(); s += 1.0)
    {
      Eigen::Vector2d cur = spline_xy->getPoint(s);
      double v = _velocity_spline->operator()(s);

      if (s > 0.)
      {
        line.setColor(0, (v + prev_v) / (2. * 3. * 2.), 0.);
        line.addLine(prev, cur);
      }
      prev = cur;
      prev_v = v;
    }
    publisher.publish();
  }

} // namespace MPCPlanner