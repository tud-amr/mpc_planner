#ifndef __MPC_PLANNER_SOLVER_H__
#define __MPC_PLANNER_SOLVER_H__

#include <mpc_planner_solver/state.h>

#include <mpc_planner_util/load_yaml.hpp>

#include <memory>

#include <Solver.h>
#include <Solver_memory.h>

#include <Eigen/Dense>

extern "C"
{
	extern solver_int32_default Solver_adtool2forces(Solver_float *x,		 /* primal vars                                         */
							 Solver_float *y,		 /* eq. constraint multiplers                           */
							 Solver_float *l,		 /* ineq. constraint multipliers                        */
							 Solver_float *p,		 /* parameters                                          */
							 Solver_float *f,		 /* objective function (scalar)                         */
							 Solver_float *nabla_f,		 /* gradient of objective function                      */
							 Solver_float *c,		 /* dynamics                                            */
							 Solver_float *nabla_c,		 /* Jacobian of the dynamics (column major)             */
							 Solver_float *h,		 /* inequality constraints                              */
							 Solver_float *nabla_h,		 /* Jacobian of inequality constraints (column major)   */
							 Solver_float *hess,		 /* Hessian (column major)                              */
							 solver_int32_default stage,	 /* stage number (0 indexed)                            */
							 solver_int32_default iteration, /* iteration number of solver                          */
							 solver_int32_default threadID /* Id of caller thread 								   */);
}

namespace MPCPlanner
{
	class Solver
	{

	protected:
		char *_solver_memory;
		Solver_mem *_solver_memory_handle;

	public:
		int _solver_id;

		Solver_params _params;
		Solver_output _output;
		Solver_info _info;

		int N;		   // Horizon length
		unsigned int nu;   // Number of control variables
		unsigned int nx;   // Differentiable variables
		unsigned int nvar; // Total variable count
		unsigned int npar; // Parameters per iteration
		double dt;

		YAML::Node _config, _parameter_map, _model_map;

		Solver(int solver_id = 0);
		void reset();
		~Solver();

		/** @brief Copy data from another solver. Does not copy solver generic parameters like the horizon N*/
		Solver &operator=(const Solver &rhs);

		char *getSolverMemory() const;
		void copySolverMemory(const Solver &other);

		void setEgoPrediction(unsigned int k, std::string &&var_name, double value);
		double getEgoPrediction(unsigned int k, std::string &&var_name);
		void setEgoPredictionPosition(unsigned int k, const Eigen::Vector2d &value);
		Eigen::Vector2d getEgoPredictionPosition(unsigned int k);

		/** @brief Set and get a solver parameter at index index of stage k */
		bool hasParameter(std::string &&parameter);
		void setParameter(int k, std::string &&parameter, double value);
		void setParameter(int k, std::string &parameter, double value);
		double getParameter(int k, std::string &&parameter);

		void setXinit(std::string &&state_name, double value);
		void setXinit(const State &state);

		void initializeWithState(const State &initial_state);
		void initializeWarmstart(const State &state, bool shift_previous_solution_forward);
		void loadWarmstart();

		void setReinitialize(bool reinitialize);

		/** @brief Solve the optimization */
		int solve();
		double getOutput(int k, std::string &&state_name) const;

		// Debugging utilities
		std::string explainExitFlag(int exitflag);
		void printIfBoundLimited() const;
		void printParameters(int k);
	};
}
#endif // __MPC_PLANNER_SOLVER_H__