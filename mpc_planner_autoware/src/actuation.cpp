#include "mpc_planner_autoware/actuation.h"

#include <mpc_planner_autoware/utils.h>

#include <mpc_planner/planner.h>

#include <mpc_planner_solver/state.h>
#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/data_visualization.h>
#include <mpc_planner_types/data_types.h>

#include <ros_tools/ros2_wrappers.h>
#include <ros_tools/convertions.h>
#include <ros_tools/spline.h>

namespace MPCPlanner
{

        void trajectoryToPath(const Planner &planner) // I don't think that this is necessary
        {
                // Construct a spline t -> x, y
                std::vector<double> x, y, t;
                int N = CONFIG["N"].as<int>();
                double dt = CONFIG["integrator_step"].as<double>();

                for (int k = 1; k < N; k++) // forward for now
                {
                        x.push_back(planner.getSolution(k, "x"));
                        y.push_back(planner.getSolution(k, "y"));
                        t.push_back((double)(k)*dt);
                }

                RosTools::Spline2D trajectory(x, y, t);
                const std::vector<double> &s = trajectory.getDistanceVector();
                visualizeSpline(trajectory, "debug", true);

                tk::spline s_to_t;
                s_to_t.set_points(s, t);

                // Fit a spline all signals over the target distance
        }

        autoware_auto_planning_msgs::msg::Trajectory outputToAutowareTrajectoryMsg(const Planner &planner)
        {
                trajectoryToPath(planner);

                auto trajectory_msg = autoware_auto_planning_msgs::msg::Trajectory();
                trajectory_msg.points.clear();

                int N = CONFIG["N"].as<int>();
                double dt = CONFIG["integrator_step"].as<double>();

                for (size_t k = 1; k < N; k++)
                {
                        trajectory_msg.points.emplace_back();
                        auto &point = trajectory_msg.points.back();

                        // Seems to be ignored by the trajectory follower
                        point.time_from_start = rclcpp::Duration::from_seconds((double)(k)*dt);

                        double angle = planner.getSolution(k, "psi");
                        Eigen::Vector2d center_pos(planner.getSolution(k, "x"),
                                                   planner.getSolution(k, "y"));

                        // We map here the center state (used in our model) to the autoware position in the center of the rear axel
                        Eigen::Vector2d autoware_pos = mapFromVehicleCenter(center_pos, angle);
                        point.pose.position.x = autoware_pos(0);
                        point.pose.position.y = autoware_pos(1);
                        point.pose.orientation = RosTools::angleToQuaternion(angle);
                        point.longitudinal_velocity_mps = planner.getSolution(k, "v");

                        // Inputs start from each k, i.e., from 0 - (N-1)
                        point.acceleration_mps2 = planner.getSolution(k, "a"); // Use the input at k (not k-1)
                        point.heading_rate_rps = planner.getSolution(k, "w");
                }

                trajectory_msg.header.frame_id = "map";
                trajectory_msg.header.stamp = GET_STATIC_NODE_POINTER()->get_clock()->now(); // Trajectory is defined from the last state for which we computed the trajectory
                return trajectory_msg;
        }

        autoware_auto_planning_msgs::msg::Trajectory backupTrajectoryToAutowareTrajectoryMsg(const State &state)
        {
                auto trajectory_msg = autoware_auto_planning_msgs::msg::Trajectory();
                trajectory_msg.points.clear();
                int N = CONFIG["N"].as<int>();
                double dt = CONFIG["integrator_step"].as<double>();

                double x = state.get("x");
                double y = state.get("y");
                double psi = state.get("psi");
                double v = state.get("v");

                double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();

                /** @note: Returning 1e-2 instead of zero prevents Autoware from choking */
                auto positive_velocity_lambda = [](double v)
                { return std::max(v, 1e-2); };

                // x = x, y = y, psi = psi
                v = positive_velocity_lambda(v - deceleration * dt); // Update for the initial state

                for (int k = 0; k < N - 1; k++)
                {
                        trajectory_msg.points.emplace_back();
                        trajectory_msg.points.back().time_from_start = rclcpp::Duration::from_seconds((double)(k + 1) * dt);

                        x += v * std::cos(psi) * dt;
                        y += v * std::sin(psi) * dt;
                        Eigen::Vector2d center_pos(x, y);

                        // We map here the center state (used in our model) to the autoware position in the center of the rear axel
                        Eigen::Vector2d autoware_pos = mapFromVehicleCenter(center_pos, psi);

                        trajectory_msg.points.back().pose.position.x = autoware_pos(0);
                        trajectory_msg.points.back().pose.position.y = autoware_pos(1);
                        trajectory_msg.points.back().pose.orientation = RosTools::angleToQuaternion(psi);
                        trajectory_msg.points.back().longitudinal_velocity_mps = v;

                        v = positive_velocity_lambda(v - deceleration * dt); // Brake with deceleration
                }

                trajectory_msg.header.frame_id = "map";
                trajectory_msg.header.stamp = GET_STATIC_NODE_POINTER()->get_clock()->now(); // Trajectory is defined from the last state for which we computed the trajectory
                return trajectory_msg;
        }
}