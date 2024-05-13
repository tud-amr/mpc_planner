#ifndef MPC_REALTIME_DATA_TYPES_H
#define MPC_REALTIME_DATA_TYPES_H

#include <mpc_planner_types/data_types.h>

#include <chrono>

namespace MPCPlanner
{

    struct RealTimeData
    {

        std::vector<Disc> robot_area;
        FixedSizeTrajectory past_trajectory;

        std::vector<DynamicObstacle> dynamic_obstacles;
        ReferencePath reference_path;
        Boundary left_bound, right_bound;

        Eigen::Vector2d goal;
        bool goal_received{false};

        // Feedback data
        double intrusion;

        std::chrono::system_clock::time_point planning_start_time;

        RealTimeData() = default;

        void reset()
        {
            // Copy data that should remain at reset
            std::vector<Disc> robot_area_copy = robot_area;

            *this = RealTimeData();

            robot_area = robot_area_copy;
            goal_received = false;
        }
    };

}
#endif