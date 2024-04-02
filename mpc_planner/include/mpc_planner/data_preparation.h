#ifndef DATA_PREPARATION_H
#define DATA_PREPARATION_H

#include <Eigen/Dense>
#include <vector>

namespace MPCPlanner
{
  struct Disc;
  struct State;
  struct DynamicObstacle;
  struct Prediction;

  std::vector<Disc> defineRobotArea(double length, double width, int n_discs);

  DynamicObstacle getDummyObstacle(const State &state);

  Prediction getConstantVelocityPrediction(const Eigen::Vector2d &position,
                                           const Eigen::Vector2d &velocity,
                                           double dt, int steps);

  void removeDistantObstacles(std::vector<DynamicObstacle> &obstacles, const State &state);
  void ensureObstacleSize(std::vector<DynamicObstacle> &obstacles, const State &state);

  void propagatePredictionUncertainty(Prediction &prediction);
  void propagatePredictionUncertainty(std::vector<DynamicObstacle> &obstacles);
} // namespace MPCPlanner

#endif // DATA_PREPARATION_H
