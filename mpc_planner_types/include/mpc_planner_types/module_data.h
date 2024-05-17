#ifndef MODULE_DATA_H
#define MODULE_DATA_H

#include <mpc_planner_types/data_types.h>

#include <vector>

namespace MPCPlanner
{
    struct ModuleData
    {
        std::vector<StaticObstacle> static_obstacles;
        int current_path_segment{-1};
    };
}

#endif // MODULE_DATA_H
