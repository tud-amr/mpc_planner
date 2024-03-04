#ifndef DATA_PREPARATION_H
#define DATA_PREPARATION_H

#include <mpc-planner-solver/state.h>

#include <mpc_planner_types/data_types.h>

#include <mpc-planner-util/parameters.h>

#include <ros_tools/logging.h>
#include <ros_tools/math.h>

#include <numeric>

inline DynamicObstacle getDummyObstacle(const State &state)
{
  return DynamicObstacle(
      -1,
      Eigen::Vector2d(state.get("x") + 100., state.get("y") + 100.),
      0.,
      0.);
}

inline Prediction getConstantVelocityPrediction(const Eigen::Vector2d &position, const Eigen::Vector2d &velocity, double dt, int steps)
{
  Prediction prediction(PredictionType::DETERMINISTIC);

  for (int i = 0; i < steps; i++)
    prediction.steps.push_back(PredictionStep(position + velocity * dt * i, 0., 0., 0.));

  return prediction;
}

inline void ensureObstacleSize(std::vector<DynamicObstacle> &obstacles, const State &state)
{
  size_t max_obstacles = CONFIG["max_obstacles"].as<int>();

  // Create an index list
  std::vector<int> indices;
  indices.resize(obstacles.size());
  std::iota(indices.begin(), indices.end(), 0);

  // If more, we sort and retrieve the closest obstacles
  if (obstacles.size() > max_obstacles)
  {
    std::vector<double> distances;
    LOG_DEBUG("Received " << obstacles.size() << " > " << max_obstacles << " obstacles. Keeping the closest.");

    Eigen::Vector2d obstacle_pos;
    Eigen::Vector2d vehicle_pos = state.getPos();

    distances.clear();
    for (auto &obstacle : obstacles)
    {

      //   if (reject_function != nullptr && reject_function(vehicle_pos, obstacle.position)) // If we should reject this                                                                                    // obstacle, push a high distance
      // distances.push_back(1e8);
      //   else
      distances.push_back(RosTools::distance(vehicle_pos, obstacle.position));
    }

    // Sort obstacles on distance
    std::sort(indices.begin(), indices.end(), [&](const int a, const int b)
              { return (distances[a] < distances[b]); });

    // Keep the closest obstacles
    std::vector<DynamicObstacle> processed_obstacles;
    processed_obstacles.clear();

    for (size_t v = 0; v < max_obstacles; v++)
      processed_obstacles.push_back(obstacles[indices[v]]);

    obstacles = processed_obstacles;
  }
  else if (obstacles.size() < max_obstacles)
  {
    LOG_DEBUG("Received " << obstacles.size() << " < " << max_obstacles << " obstacles. Adding dummies.");

    for (size_t cur_size = obstacles.size(); cur_size < max_obstacles; cur_size++)
    {
      obstacles.push_back(getDummyObstacle(state));

      auto &obstacle = obstacles.back();
      obstacle.prediction = getConstantVelocityPrediction(obstacle.position,
                                                          Eigen::Vector2d(0., 0.),
                                                          CONFIG["integrator_step"].as<double>(),
                                                          CONFIG["N"].as<int>());
    }
  }

  LOG_DEBUG("Obstacle size (after processing) is: " << obstacles.size());
}

inline void propagatePredictionUncertainty(Prediction &prediction)
{
  if (prediction.type != PredictionType::GAUSSIAN)
  {
    LOG_WARN("Cannot propagate uncertainty for predictions that are not GAUSSIAN");
    return;
  }

  double dt = CONFIG["integrator_step"].as<double>();
  double major = 0.;
  double minor = 0.;

  for (int k = 0; k < CONFIG["N"].as<int>(); k++)
  {
    major = std::sqrt(std::pow(major, 2.0) + std::pow(prediction.steps[k].major_radius * dt, 2.));
    minor = std::sqrt(std::pow(minor, 2.0) + std::pow(prediction.steps[k].minor_radius * dt, 2.));
    prediction.steps[k].major_radius = major;
    prediction.steps[k].minor_radius = minor;
  }
}

inline void propagatePredictionUncertainty(std::vector<DynamicObstacle> &obstacles)
{
  for (auto &obstacle : obstacles)
    propagatePredictionUncertainty(obstacle.prediction);
}

#endif // DATA_PREPARATION_H
