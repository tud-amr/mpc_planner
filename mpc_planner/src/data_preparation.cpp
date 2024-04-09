#include <mpc_planner/data_preparation.h>

#include <mpc_planner_solver/state.h>

#include <mpc_planner_types/data_types.h>

#include <mpc_planner_util/parameters.h>

#include <ros_tools/logging.h>
#include <ros_tools/math.h>

#include <numeric>

namespace MPCPlanner
{
        std::vector<Disc> defineRobotArea(double length, double width, int n_discs)
        {
                // Where is the center w.r.t. the back of the vehicle
                double center_offset = length / 2.; // Could become a parameter
                double radius = width / 2.;

                std::vector<Disc> robot_area;
                ROSTOOLS_ASSERT(n_discs > 0, "Trying to create a collision region with less than a disc");

                if (n_discs == 1)
                {
                        robot_area.emplace_back(0., radius);
                }
                else
                {

                        for (int i = 0; i < n_discs; i++)
                        {
                                if (i == 0)
                                        robot_area.emplace_back(-center_offset + radius, radius); // First disc at the back of the car
                                else if (i == n_discs - 1)
                                        robot_area.emplace_back(-center_offset + length - radius, radius); // Last disc at the front of the car
                                else
                                        robot_area.emplace_back(-center_offset + radius +
                                                                    (double)i * (length - 2. * radius) / ((double)(n_discs - 1.)),
                                                                radius); // Other discs in between
                                LOG_VALUE("offset", robot_area.back().offset);
                                LOG_VALUE("radius", robot_area.back().radius);
                        }
                }

                return robot_area;
        }

        DynamicObstacle getDummyObstacle(const State &state)
        {
                return DynamicObstacle(
                    -1,
                    Eigen::Vector2d(state.get("x") + 100., state.get("y") + 100.),
                    0.,
                    0.);
        }

        Prediction getConstantVelocityPrediction(const Eigen::Vector2d &position, const Eigen::Vector2d &velocity, double dt, int steps)
        {
                Prediction prediction;
                double noise = 0.;
                if (CONFIG["probabilistic"]["enable"].as<bool>())
                {
                        prediction = Prediction(PredictionType::GAUSSIAN);
                        noise = 0.3;
                }
                else
                {
                        prediction = Prediction(PredictionType::DETERMINISTIC);
                }

                for (int i = 0; i < steps; i++)
                        prediction.modes[0].push_back(PredictionStep(position + velocity * dt * i, 0., noise, noise));

                if (CONFIG["probabilistic"]["enable"].as<bool>())
                        propagatePredictionUncertainty(prediction);

                return prediction;
        }

        void removeDistantObstacles(std::vector<DynamicObstacle> &obstacles, const State &state)
        {
                std::vector<DynamicObstacle> nearby_obstacles;

                const Eigen::Vector2d pos = state.getPos();
                for (auto &obstacle : obstacles)
                {
                        if (RosTools::distance(pos, obstacle.position) < CONFIG["max_obstacle_distance"].as<double>())
                                nearby_obstacles.push_back(obstacle);
                }

                obstacles = nearby_obstacles;
        }

        void ensureObstacleSize(std::vector<DynamicObstacle> &obstacles, const State &state)
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
                        LOG_MARK("Received " << obstacles.size() << " > " << max_obstacles << " obstacles. Keeping the closest.");

                        Eigen::Vector2d obstacle_pos;
                        Eigen::Vector2d vehicle_pos = state.getPos();

                        distances.clear();
                        for (auto &obstacle : obstacles)
                        {
                                double dist;
                                double min_dist = 1e5;

                                //   if (reject_function != nullptr && reject_function(vehicle_pos, obstacle.position)) // If we should reject this                                                                                    // obstacle, push a high distance
                                // distances.push_back(1e8);
                                //   else
                                // Eigen::Vector2d obstacle_pos = obstacle.position;
                                // Eigen::Vector2d obstacle_pos = obstacle.position;
                                Eigen::Vector2d direction(std::cos(state.get("psi")), std::sin(state.get("psi")));
                                for (int k = 0; k < CONFIG["N"].as<int>(); k++)
                                {

                                        dist = std::pow(1.01, (double)k) *
                                               RosTools::distance(
                                                   obstacle.prediction.modes[0][k].position,
                                                   state.getPos() + state.get("v") * (double)k * direction);

                                        if (dist < min_dist)
                                                min_dist = dist;
                                }
                                distances.push_back(min_dist); // RosTools::distance(vehicle_pos, obstacle.position));
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
                        LOG_MARK("Received " << obstacles.size() << " < " << max_obstacles << " obstacles. Adding dummies.");

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

                LOG_MARK("Obstacle size (after processing) is: " << obstacles.size());
        }

        void propagatePredictionUncertainty(Prediction &prediction)
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
                        major = std::sqrt(std::pow(major, 2.0) + std::pow(prediction.modes[0][k].major_radius * dt, 2.));
                        minor = std::sqrt(std::pow(minor, 2.0) + std::pow(prediction.modes[0][k].minor_radius * dt, 2.));
                        prediction.modes[0][k].major_radius = major;
                        prediction.modes[0][k].minor_radius = minor;
                }
        }

        void propagatePredictionUncertainty(std::vector<DynamicObstacle> &obstacles)
        {
                for (auto &obstacle : obstacles)
                        propagatePredictionUncertainty(obstacle.prediction);
        }
}