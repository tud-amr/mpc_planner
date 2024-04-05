#ifndef AUTOWARE_UTILS_H
#define AUTOWARE_UTILS_H

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <Eigen/Dense>

using autoware_auto_planning_msgs::msg::PathWithLaneId;

namespace MPCPlanner
{
        struct RealTimeData;

        Eigen::Vector2d mapToVehicleCenter(const Eigen::Vector2d &autoware_pos, const double angle);

        Eigen::Vector2d mapFromVehicleCenter(const Eigen::Vector2d &center_pos, const double angle);

        bool isPathTheSame(PathWithLaneId::SharedPtr msg, const RealTimeData &data);
}
#endif // AUTOWARE_UTILS_H
