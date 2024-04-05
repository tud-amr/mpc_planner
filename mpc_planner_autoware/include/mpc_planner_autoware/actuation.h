#ifndef AUTOWARE_ACTUATION_H
#define AUTOWARE_ACTUATION_H

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

namespace MPCPlanner
{

        class Trajectory;
        class Planner;
        struct State;

        autoware_auto_planning_msgs::msg::Trajectory outputToAutowareTrajectoryMsg(const Planner &planner);
        autoware_auto_planning_msgs::msg::Trajectory backupTrajectoryToAutowareTrajectoryMsg(const State &state);

        void trajectoryToPath(const Planner &planner);
}

#endif // AUTOWARE_ACTUATION_H
