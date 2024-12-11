#ifndef __SCENARIO_CONSTRAINTS_H_
#define __SCENARIO_CONSTRAINTS_H_

#include <mpc_planner_modules/controller_module.h>

#include <scenario_module/scenario_module.h>

namespace MPCPlanner
{

  class ScenarioConstraints : public ControllerModule
  {
  public:
    ScenarioConstraints(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
    void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    void onDataReceived(RealTimeData &data, std::string &&data_name) override;
    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    // Optimizes multiple trajectories in parallel
    int optimize(State &state, const RealTimeData &data, ModuleData &module_data) override;

    void visualize(const RealTimeData &data, const ModuleData &module_data) override;

  private:
    double _planning_time;

    /** @brief a struct to collect parallel scenario optimizations */
    struct ScenarioSolver
    {
      ScenarioModule::ScenarioModule scenario_module; // Declared here for memory continuity
      std::shared_ptr<Solver> solver;

      // Outputs
      int exit_code{-1};
      ScenarioModule::ScenarioStatus status{ScenarioModule::ScenarioStatus::SUCCESS};
      ScenarioModule::SupportSubsample support;

      ScenarioSolver(int id);
    };
    std::vector<std::unique_ptr<ScenarioSolver>> _scenario_solvers;

    ScenarioSolver *_best_solver;

    int sequentialScenarioIterations();
  };
}
#endif // __SCENARIO_CONSTRAINTS_H_
