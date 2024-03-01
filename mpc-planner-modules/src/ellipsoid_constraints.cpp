#include "mpc-planner-modules/ellipsoid_constraints.h"

#include <mpc-planner-util/parameters.h>

#include <ros_planner_utils/visuals.h>
#include <ros_planner_utils/math.h>

#include <algorithm>

namespace MPCPlanner
{
  EllipsoidConstraints::EllipsoidConstraints(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "ellipsoid_constraints")
  {
    LOG_INFO("Initializing EllipsoidConstraints Module");
  }

  void EllipsoidConstraints::update(State &state, const RealTimeData &data)
  {
    (void)state;
    (void)data;
  }

  void EllipsoidConstraints::setParameters(const RealTimeData &data, int k)
  {

    _solver->setParameter(k, "ego_disc_radius", CONFIG["robot_radius"].as<double>());
    for (int d = 0; d < CONFIG["n_discs"].as<int>(); d++)
      _solver->setParameter(k, "ego_disc_" + std::to_string(d) + "_offset", 0.); /** @todo Fix offsets! */

    for (size_t i = 0; i < data.dynamic_obstacles.size(); i++)
    {
      const auto &obstacle = data.dynamic_obstacles[i];
      /** @note The first prediction step is index 1 of the optimization problem, i.e., k-1 maps to the predictions for this stage */
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_x", obstacle.prediction.steps[k - 1].position(0));
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_y", obstacle.prediction.steps[k - 1].position(1));
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_psi", obstacle.prediction.steps[k - 1].angle);

      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_r", obstacle.radius);

      if (obstacle.prediction.type == PredictionType::DETERMINISTIC)
      {
        _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_major", 0.);
        _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_minor", 0.);
        _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_chi", 1.0);
      }
      else if (obstacle.prediction.type == PredictionType::GAUSSIAN)
      {
        double chi = RosTools::ExponentialQuantile(0.5, 1.0 - CONFIG["probabilistic"]["risk"].as<double>());

        _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_major", obstacle.prediction.steps[k - 1].major_radius);
        _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_minor", obstacle.prediction.steps[k - 1].minor_radius);
        _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_chi", chi);
      }
    }
  }

  bool EllipsoidConstraints::isDataReady(const RealTimeData &data, std::string &missing_data)
  {
    if (data.dynamic_obstacles.size() != CONFIG["max_obstacles"].as<unsigned int>())
    {
      missing_data += "Obstacles ";
      return false;
    }

    for (size_t i = 0; i < data.dynamic_obstacles.size(); i++)
    {
      if (data.dynamic_obstacles[i].prediction.steps.empty())
      {
        missing_data += "Obstacle Prediction ";
        return false;
      }

      if (data.dynamic_obstacles[i].prediction.type != PredictionType::GAUSSIAN && data.dynamic_obstacles[i].prediction.type != PredictionType::DETERMINISTIC)
      {
        missing_data += "Obstacle Prediction (Type is incorrect) ";
        return false;
      }
    }

    return true;
  }

  void EllipsoidConstraints::visualize(const RealTimeData &data)
  {
    (void)data;
    //   if (_spline.get() == nullptr)
    //     return;

    //   // Visualize the current points
    //   auto &publisher_current = VISUALS.getPublisher(_name + "/current");
    //   auto &cur_point = publisher_current.getNewPointMarker("CUBE");
    //   cur_point.setColorInt(10);
    //   cur_point.setScale(0.3, 0.3, 0.3);
    //   cur_point.addPointMarker(_spline->getPoint(_spline->getStartOfSegment(_closest_segment)), 0.0);
    //   publisher_current.publish();

    //   // Visualize the points
    //   auto &publisher_points = VISUALS.getPublisher(_name + "/points");
    //   auto &point = publisher_points.getNewPointMarker("CYLINDER");
    //   point.setColor(0., 0., 0.);
    //   point.setScale(0.15, 0.15, 0.05);

    //   for (size_t p = 0; p < data.reference_path.x.size(); p++)
    //     point.addPointMarker(Eigen::Vector3d(data.reference_path.x[p], data.reference_path.y[p], 0.1));
    //   publisher_points.publish();

    //   // Visualize the path
    //   auto &publisher_path = VISUALS.getPublisher(_name + "/path");
    //   auto &line = publisher_path.getNewLine();
    //   line.setColorInt(5);
    //   line.setScale(0.1);

    //   Eigen::Vector2d p;
    //   for (double s = 0.; s < _spline->length(); s += 1.)
    //   {
    //     if (s > 0.)
    //       line.addLine(p, _spline->getPoint(s));

    //     p = _spline->getPoint(s);
    //   }

    //   publisher_path.publish();
  }

} // namespace MPCPlanner
