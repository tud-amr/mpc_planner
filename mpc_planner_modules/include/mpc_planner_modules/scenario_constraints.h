#ifndef __SCENARIO_CONSTRAINTS_H_
#define __SCENARIO_CONSTRAINTS_H_

#include <mpc_planner_modules/controller_module.h>

#include <lmpcc_scenario_module/lmpcc_scenario_module.h>

#include <list>

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

    int optimize(State &state, const RealTimeData &data, ModuleData &module_data) override; // Default: no custom optimization

    void visualize(const RealTimeData &data, const ModuleData &module_data) override;

  private:
    // std::unique_ptr<ScenarioModule::ScenarioModule> _scenario_module;

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

    // ScenarioSolveStatus solve_status_;

    int sequentialScenarioIterations();
  };
}
#endif // __SCENARIO_CONSTRAINTS_H_
