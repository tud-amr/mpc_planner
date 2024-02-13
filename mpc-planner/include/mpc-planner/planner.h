#ifndef MPC_PLANNER_H
#define MPC_PLANNER_H

#include <yaml-cpp/yaml.h>

#include <memory>

namespace MPCPlanner
{
    class RealTimeData;
    class State;
    class ControllerModule;
    class Solver;

    class Planner
    {
    public:
        Planner(const YAML::Node &config);

    public:
        void solveMPC(const State &state, const RealTimeData &data);

        double getSolution(int k, std::string &&var_name);

    private:
        // std::shared_ptr<YAML::Node> _config;
        YAML::Node _config;

        std::shared_ptr<Solver> _solver;

        std::vector<std::shared_ptr<ControllerModule>> _modules;
    };

};

#endif