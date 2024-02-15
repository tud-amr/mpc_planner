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

#include <mpc-planner-types/controller_module.h>

namespace MPCPlanner
{
  class MPCBaseModule : public ControllerModule
  {
  public:
    MPCBaseModule(std::shared_ptr<Solver> solver);

  public:
    virtual void update(const RealTimeData &data) override;

    virtual void setParameters(const RealTimeData &data, int k) override;

  private:
  };
};

#endif // __MPC_BASE_H__