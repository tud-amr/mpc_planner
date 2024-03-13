#ifndef MPC_REALTIME_DATA_TYPES_H
#define MPC_REALTIME_DATA_TYPES_H

#include <mpc_planner_types/data_types.h>

namespace MPCPlanner
{

    struct RealTimeData
    {

        std::vector<Disc> robot_area;

        std::vector<DynamicObstacle> dynamic_obstacles;
        ReferencePath reference_path;

        Eigen::Vector2d goal;
        bool goal_received{false};

        RealTimeData() = default;

        void reset()
        {
            // Copy data that should remain at reset
            std::vector<Disc> robot_area_copy = robot_area;

            *this = RealTimeData();

            robot_area = robot_area_copy;
        }
    };


}
#endif