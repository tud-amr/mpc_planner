
#include <mpc_planner_modules/goal_module.h>

#include <ros_tools/visuals.h>
#include <mpc_planner_util/parameters.h>

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
    }

    void GoalModule::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
    {
        if (k == 0)
            LOG_DEBUG("Goal Module::setParameters()");

        // Set the parameters for the solver
        _solver->setParameter(k, "goal_x", data.goal(0));
        _solver->setParameter(k, "goal_y", data.goal(1));

        _solver->setParameter(k, "goal_weight", CONFIG["weights"]["goal"].as<double>());
    }

    bool GoalModule::isDataReady(const RealTimeData &data, std::string &missing_data)
    {
        if (!data.goal_received)
            missing_data += "Goal ";

        return data.goal_received;
    }

    void GoalModule::visualize(const RealTimeData &data, const ModuleData &module_data)
    {
        if (!data.goal_received)
            return;

        LOG_DEBUG("GoalModule::visualize()");
        auto &publisher = VISUALS.getPublisher(_name);
        auto &sphere = publisher.getNewPointMarker("SPHERE");

        sphere.setColorInt(5);
        sphere.setScale(0.4, 0.4, 0.4);
        sphere.addPointMarker(data.goal, 0.0);

        publisher.publish();
    }
};