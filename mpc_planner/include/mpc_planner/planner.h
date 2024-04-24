#ifndef MPC_PLANNER_H
#define MPC_PLANNER_H

#include <mpc_planner_types/data_types.h>
#include <mpc_planner_types/module_data.h>

#include <memory>
#include <vector>

namespace MPCPlanner
{
    class RealTimeData;
    class State;
    class ControllerModule;
    class Solver;
    class ExperimentUtil;

    struct PlannerOutput
    {
        Trajectory trajectory;
        bool success{false};

        PlannerOutput(double dt, int N) : trajectory(dt, N) {}

        PlannerOutput() = default;
    };

    class Planner
    {
    public:
        Planner();

    public:
        PlannerOutput solveMPC(State &state, RealTimeData &data);
        double getSolution(int k, std::string &&var_name) const;

        void onDataReceived(RealTimeData &data, std::string &&data_name);

        void saveData(State &state, RealTimeData &data);
        void visualize(const State &state, const RealTimeData &data);

        void reset(State &state, RealTimeData &data, bool success = true);

        bool isObjectiveReached(const State &state, const RealTimeData &data) const;

    private:
        bool _is_data_ready{false};
        bool _is_first_run{true};

        std::shared_ptr<Solver> _solver;
        std::shared_ptr<ExperimentUtil> _experiment_util;
        PlannerOutput _output;

        ModuleData _module_data;

        std::vector<std::shared_ptr<ControllerModule>> _modules;
    };

}

#endif