/**
 * @file mpc_base.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief A base module for updating weights of an MPC controller. Weights are configured in the python solver
 * generation.
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __MPC_BASE_H__
#define __MPC_BASE_H__

#include <mpc_planner_modules/controller_module.h>

namespace MPCPlanner
{
  class MPCBaseModule : public ControllerModule
  {
  public:
    MPCBaseModule(std::shared_ptr<Solver> solver);

  public:
    virtual void update(State &state, const RealTimeData &data, ModuleData &module_data) override;

    virtual void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

  private:
    std::vector<std::string> _weight_names;
  };
}

#endif // __MPC_BASE_H__