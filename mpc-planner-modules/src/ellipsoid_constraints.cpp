#include "mpc-planner-modules/ellipsoid_constraints.h"

#include <mpc-planner-util/parameters.h>
#include <mpc-planner-util/visuals.h>

#include <algorithm>

namespace MPCPlanner
{
  EllipsoidConstraints::EllipsoidConstraints(std::shared_ptr<Solver> solver)
      : ControllerModule(solver, ModuleType::OBJECTIVE, "contouring")
  {
    LOG_INFO("Initializing EllipsoidConstraints Module");
  }

  void EllipsoidConstraints::update(State &state, const RealTimeData &data)
  {
  
  }

  void EllipsoidConstraints::setParameters(const RealTimeData &data, int k)
  {
    
//     ellipsoid_obst_0_x
//  36: ellipsoid_obst_0_y
//  37: ellipsoid_obst_0_psi
//  38: ellipsoid_obst_0_major
//  39: ellipsoid_obst_0_minor
//  40: ellipsoid_obst_0_chi
//  41: ellipsoid_obst_0_r


    for(int i = 0; i < 8; i++)
    {
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_x", 3.0);
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_y", 2.0);
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_psi", 0.0);
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_major", 1.0);
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_minor", 1.0);
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_chi", 1.0);
      _solver->setParameter(k, "ellipsoid_obst_" + std::to_string(i) + "_r", 0.5);
    }
  }

  bool EllipsoidConstraints::isDataReady(const RealTimeData &data, std::string &missing_data)
  {
    return true;
    // if (data.reference_path.x.empty())
    //   missing_data += "Reference Path ";

    // return !data.reference_path.x.empty();
  }

  void EllipsoidConstraints::visualize(const RealTimeData &data)
  {
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

}; // namespace MPCPlanner
  