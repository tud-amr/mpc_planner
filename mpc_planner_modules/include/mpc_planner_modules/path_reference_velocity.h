/**
 * @file path_reference_velocity.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief A module for tracking a velocity along the path
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __REFERENCE_PATH_VELOCITY_H__
#define __REFERENCE_PATH_VELOCITY_H__

#include <mpc_planner_modules/controller_module.h>

namespace tk
{
  class spline;
}

namespace MPCPlanner
{
  class PathReferenceVelocity : public ControllerModule
  {
  public:
    PathReferenceVelocity(std::shared_ptr<Solver> solver);

  public:
    virtual void update(State &state, const RealTimeData &data, ModuleData &module_data) override;

    virtual void onDataReceived(RealTimeData &data, std::string &&data_name) override;

    virtual void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    virtual void visualize(const RealTimeData &data, const ModuleData &module_data) override;

  private:
    std::shared_ptr<tk::spline> _velocity_spline;
    int _n_segments;
  };
}

#endif // __REFERENCE_PATH_VELOCITY_H__