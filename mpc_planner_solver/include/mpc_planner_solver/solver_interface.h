#ifndef __MPC_PLANNER_SOLVER_H__
#define __MPC_PLANNER_SOLVER_H__

#ifdef MPC_PLANNER_ROS // ACADOS_SOLVER

#include <mpc_planner_solver/acados_solver_interface.h>

#else

#include <mpc_planner_solver/forces_solver_interface.h>

#endif

#endif