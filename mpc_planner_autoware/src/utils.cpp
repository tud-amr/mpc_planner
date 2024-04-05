#include "mpc_planner_autoware/utils.h"

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_types/realtime_data.h>

#include <algorithm>

namespace MPCPlanner
{
        Eigen::Vector2d mapToVehicleCenter(const Eigen::Vector2d &autoware_pos, const double angle)
        {
                double length_div_2 = CONFIG["robot"]["length"].as<double>() / 2.;
                double com_to_back = CONFIG["robot"]["com_to_back"].as<double>();
                return Eigen::Vector2d(
                    autoware_pos(0) + std::cos(angle) * (length_div_2 - com_to_back),
                    autoware_pos(1) + std::sin(angle) * (length_div_2 - com_to_back));
        }

        Eigen::Vector2d mapFromVehicleCenter(const Eigen::Vector2d &center_pos, const double angle)
        {
                double length_div_2 = CONFIG["robot"]["length"].as<double>() / 2.;
                double com_to_back = CONFIG["robot"]["com_to_back"].as<double>();
                return Eigen::Vector2d(
                    center_pos(0) - std::cos(angle) * (length_div_2 - com_to_back),
                    center_pos(1) - std::sin(angle) * (length_div_2 - com_to_back));
        }

        bool isPathTheSame(PathWithLaneId::SharedPtr msg, const RealTimeData &data)
        {
                // Check if the path is the same
                if (data.reference_path.x.size() != msg->points.size())
                {
                        return false;
                }

                // Check up to the first two points
                int num_points = std::min(2, (int)data.reference_path.x.size());
                for (int i = 0; i < num_points; i++)
                {
                        if (!data.reference_path.pointInPath(i,
                                                             msg->points[i].point.pose.position.x,
                                                             msg->points[i].point.pose.position.y))
                        {
                                return false;
                        }
                }
                return true;
        }
}