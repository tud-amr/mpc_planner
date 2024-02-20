#include "mpc-planner-modules/linearized_constraints.h"

#include <mpc-planner-util/parameters.h>
#include <mpc-planner-util/visuals.h>

#include <ros_tools/helpers.h>

#include <algorithm>

namespace MPCPlanner
{
  LinearizedConstraints::LinearizedConstraints(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "linearized_constraints")
  {
    LOG_INFO("Initializing LinearizedConstraints Module");

    _a1.resize(CONFIG["n_discs"].as<int>());
    _a2.resize(CONFIG["n_discs"].as<int>());
    _b.resize(CONFIG["n_discs"].as<int>());
    for (int d = 0; d < CONFIG["n_discs"].as<int>(); d++)
    {
      std::cout << d << std::endl;
      std::cout << CONFIG["N"].as<int>() << std::endl;
      // Initialize arrays for the constraints
      _a1[d].resize(CONFIG["N"].as<int>());
      _a2[d].resize(CONFIG["N"].as<int>());
      _b[d].resize(CONFIG["N"].as<int>());
      for (int k = 0; k < CONFIG["N"].as<int>(); k++)
      {

        _a1[d][k] = Eigen::ArrayXd(CONFIG["max_obstacles"].as<int>());
        _a2[d][k] = Eigen::ArrayXd(CONFIG["max_obstacles"].as<int>());
        _b[d][k] = Eigen::ArrayXd(CONFIG["max_obstacles"].as<int>());
      }
    }

    _num_obstacles = 0;
  }

  void LinearizedConstraints::update(State &state, const RealTimeData &data)
  {
    // Thread safe
    std::vector<DynamicObstacle> copied_obstacles = data.dynamic_obstacles;
    _num_obstacles = copied_obstacles.size();

    // for (auto &obstacle : copied_obstacles)
    // {
    //   for (auto &disc : obstacle.discs_)
    //     disc.radius = 1e-3; // We fully relax the constraints here
    // }

    // For all stages
    for (int k = 0; k < _solver->N; k++)
    {
      // for (int disc_id = 0; disc_id < CONFIG["n_discs"].as<int>(); disc_id++)
      // {
      // Disc &disc = vehicle_prediction[k].discs_[disc_id];
      // Eigen::Vector2d pos = vehicle_prediction[k].discs_[disc_id].AsVector2d(); // vehicle_->discs_[0].poses_[k];

      // Eigen::Vector2d pos = state.getPos();
      Eigen::Vector2d pos(_solver->getVar(k, "x"), _solver->getVar(k, "y")); // k+1?
      /** @todo Disc position */

      // Ensure that the vehicle position is collision-free
      // ProjectToSafety(k, pos, copied_obstacles, solver_interface->area_->DiscRadius());

      // If we projected, load the updated position back into the solver
      // disc.SetPosition(pos);                                                                 // Save the projected position in the Disc
      // Eigen::Vector2d associated_vehicle_pos = vehicle_prediction[k].PositionFromDisc(disc); // Translate the disc to the vehicle position
      // vehicle_prediction[k].SetPosition(associated_vehicle_pos);                             // Save the vehicle position
      // solver_interface->LoadVehiclePredictionsToInitialPlan(vehicle_prediction);

      /** @todo Radius */
      double radius = 1e-3;

      // For all obstacles
      for (size_t obs_id = 0; obs_id < copied_obstacles.size(); obs_id++)
      {
        const auto &copied_obstacle = copied_obstacles[obs_id];
        const Eigen::Vector2d &obstacle_pos = copied_obstacle.prediction.steps[k].position;

        double diff_x = obstacle_pos(0) - pos(0);
        double diff_y = obstacle_pos(1) - pos(1);

        double d = (obstacle_pos - pos).norm();

        // Compute the components of A for this obstacle (normalized normal vector)
        _a1[0][k](obs_id) = diff_x / d;
        _a2[0][k](obs_id) = diff_y / d;

        // Compute b (evaluate point on the collision circle)
        _b[0][k](obs_id) = _a1[0][k](obs_id) * obstacle_pos(0) +
                           _a2[0][k](obs_id) * obstacle_pos(1) -
                           (radius + CONFIG["robot_radius"].as<double>());
        // }
      }
    }
  }

  void LinearizedConstraints::setParameters(const RealTimeData &data, int k)
  {
    for (int d = 0; d < CONFIG["n_discs"].as<int>(); d++)
      _solver->setParameter(k, "ego_disc_" + std::to_string(d) + "_offset", 0.); /** @todo Fix offsets! */

    for (size_t i = 0; i < data.dynamic_obstacles.size(); i++)
    {
      _solver->setParameter(k, "lin_constraint_" + std::to_string(i) + "_a1", _a1[0][k](i));
      _solver->setParameter(k, "lin_constraint_" + std::to_string(i) + "_a2", _a2[0][k](i));
      _solver->setParameter(k, "lin_constraint_" + std::to_string(i) + "_b", _b[0][k](i));
    }
  }

  bool LinearizedConstraints::isDataReady(const RealTimeData &data, std::string &missing_data)
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

      if (data.dynamic_obstacles[i].prediction.type != PredictionType::DETERMINISTIC)
      {
        missing_data += "Obstacle Prediction (type must be deterministic) ";
        return false;
      }
    }

    return true;
  }

  void LinearizedConstraints::visualize(const RealTimeData &data)
  {
    for (int k = 0; k < _solver->N; k++)
    {
      for (size_t i = 0; i < data.dynamic_obstacles.size(); i++)
        visualizeLinearConstraint(_a1[0][k](i), _a2[0][k](i), _b[0][k](i), k, _solver->N, _name,
                                  k == _solver->N - 1 && i == data.dynamic_obstacles.size() - 1); // Publish at the end
    }
  }

} // namespace MPCPlanner
