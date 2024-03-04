#ifndef MPC_REALTIME_DATA_TYPES_H
#define MPC_REALTIME_DATA_TYPES_H

#include <mpc_planner_types/data_types.h>

namespace MPCPlanner
{

    struct RealTimeData
    {

        std::vector<DynamicObstacle> dynamic_obstacles;
        ReferencePath reference_path;

        Eigen::Vector2d goal;
        bool goal_received{false};

        RealTimeData() = default;
    };
}
#endif