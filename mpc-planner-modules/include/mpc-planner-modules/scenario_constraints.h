#ifndef __SCENARIO_CONSTRAINTS_H_
#define __SCENARIO_CONSTRAINTS_H_

#include <mpc-planner-modules/controller_module.h>

#include <lmpcc_scenario_module/lmpcc_scenario_module.h>

namespace MPCPlanner
{

  class ScenarioConstraints : public ControllerModule
  {
  public:
    ScenarioConstraints(std::shared_ptr<Solver> solver);

  public:
    void update(State &state, const RealTimeData &data) override;
    void setParameters(const RealTimeData &data, int k) override;

    void onDataReceived(RealTimeData &data, std::string &&data_name) override;
    bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

    int optimize(State &state, const RealTimeData &data) override; // Default: no custom optimization

    void visualize(const RealTimeData &data) override;

  private:
    std::unique_ptr<ScenarioModule::ScenarioModule> _scenario_module;
    // struct ScenarioSolver
    // {
    //   std::unique_ptr<ScenarioModule::ScenarioModule> scenario_module_;

    //   std::unique_ptr<Solver> solver;

    //   // Outputs
    //   int exit_code;
    //   ScenarioModule::ScenarioStatus status;
    //   ScenarioModule::SupportSubsample support, removed_scenarios;
    // };
    // std::vector<ScenarioSolver> solvers_;

    // ScenarioSolveStatus solve_status_;

    int sequentialScenarioIterations();
  };
}
#endif // __SCENARIO_CONSTRAINTS_H_
