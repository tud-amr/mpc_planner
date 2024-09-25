
#include <mpc_planner_modules/goal_module.h>

#include <mpc_planner_util/parameters.h>

#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <ros_tools/visuals.h>
#include <ros_tools/math.h>

namespace MPCPlanner
{

    GoalModule::GoalModule(std::shared_ptr<Solver> solver)
        : ControllerModule(ModuleType::OBJECTIVE, solver, "goal_module")
    {
        LOG_INITIALIZE("Goal Tracking");
        LOG_INITIALIZED();
    }

    void GoalModule::update(State &state, const RealTimeData &data, ModuleData &module_data)
    {
        (void)state;
        (void)data;
        (void)module_data;
    }

    void GoalModule::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
    {
        (void)module_data;
        if (k == 0)
            LOG_MARK("Goal Module::setParameters()");

        setSolverParameterGoalX(k, _solver->_params, data.goal(0));
        setSolverParameterGoalY(k, _solver->_params, data.goal(1));
        setSolverParameterGoalWeight(k, _solver->_params, CONFIG["weights"]["goal"].as<double>());
    }

    bool GoalModule::isDataReady(const RealTimeData &data, std::string &missing_data)
    {
        if (!data.goal_received)
            missing_data += "Goal ";

        return data.goal_received;
    }

    bool GoalModule::isObjectiveReached(const State &state, const RealTimeData &data)
    {
        (void)data;

        if (!data.goal_received)
            return false;

        // Check if we reached the goal
        return RosTools::distance(state.getPos(), data.goal) < 1.0;
    }

    void GoalModule::visualize(const RealTimeData &data, const ModuleData &module_data)
    {
        (void)module_data;
        if (!data.goal_received)
            return;

        auto &publisher = VISUALS.getPublisher(_name);
        auto &sphere = publisher.getNewPointMarker("SPHERE");

        sphere.setColorInt(5);
        sphere.setScale(0.4, 0.4, 0.4);
        sphere.addPointMarker(data.goal, 0.0);

        publisher.publish();
    }
}