#include "mpc_planner_solver/solver_interface.h"

#include <mpc_planner_util/parameters.h>

#include <ros_tools/logging.h>

#include "mpc_planner_generated.h"

extern "C"
{
	Solver_extfunc extfunc_eval_ = &Solver_adtool2forces;
}

namespace MPCPlanner
{
	Solver::Solver(int solver_id)
	{
		_solver_id = solver_id;
		_solver_memory = (char *)malloc(Solver_get_mem_size());
		_solver_memory_handle = Solver_external_mem(_solver_memory, _solver_id, Solver_get_mem_size());
		loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "solver_settings"), _config);
		loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "parameter_map"), _parameter_map);
		loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "model_map"), _model_map);
		N = _config["N"].as<unsigned int>();
		nu = _config["nu"].as<unsigned int>();
		nx = _config["nx"].as<unsigned int>();
		nvar = _config["nvar"].as<unsigned int>();
		npar = _config["npar"].as<unsigned int>();
		dt = CONFIG["integrator_step"].as<double>();
		reset();
	}

	void Solver::reset()
	{
		for (long int i = 0; i < *(&_params.all_parameters + 1) - _params.all_parameters; i++)
			_params.all_parameters[i] = 0.0;

		for (long int i = 0; i < *(&_params.xinit + 1) - _params.xinit; i++)
			_params.xinit[i] = 0.0;

		for (size_t i = 0; i < N * nvar; i++)
			_params.x0[i] = 0.0;
	}

	Solver::~Solver()
	{
		free(_solver_memory);
	}

	Solver &Solver::operator=(const Solver &rhs)
	{
		_params = rhs._params;

		return *this;
	}

	char *Solver::getSolverMemory() const { return _solver_memory; }

	void Solver::copySolverMemory(const Solver &other)
	{
		memcpy(_solver_memory, other.getSolverMemory(), Solver_get_mem_size());
	}

	bool Solver::hasParameter(std::string &&parameter)
	{
		return _parameter_map[parameter].IsDefined();
	}

	void Solver::setParameter(int k, std::string &&parameter, double value)
	{
		_params.all_parameters[k * npar + _parameter_map[parameter].as<int>()] = value;
	}

	void Solver::setParameter(int k, std::string &parameter, double value)
	{
		_params.all_parameters[k * npar + _parameter_map[parameter].as<int>()] = value;
	}

	double Solver::getParameter(int k, std::string &&parameter)
	{
		return _params.all_parameters[k * npar + _parameter_map[parameter].as<int>()];
	}

	void Solver::setXinit(std::string &&state_name, double value)
	{
		_params.xinit[_model_map[state_name][1].as<int>() - nu] = value;
	}

	void Solver::setXinit(const State &state)
	{
		for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it)
		{
			if (it->second[0].as<std::string>() == "x")
			{
				// it->first.as<std::string>() is the variable name
				setXinit(
					it->first.as<std::string>(),
					state.get(it->first.as<std::string>()));
			}
		}
	}

	// Load the state in each instance
	void Solver::initializeWithState(const State &initial_state)
	{
		for (int k = 0; k < N; k++) // For all timesteps
		{
			for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it) // For all inputs and states
			{
				setEgoPrediction(k, it->first.as<std::string>(), initial_state.get(it->first.as<std::string>()));
			}
		}
	}

	void Solver::initializeWithBraking(const State &initial_state)
	{
		initializeWithState(initial_state); // Initialize all variables

		double x, y, psi, v, a;
		double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();

		x = initial_state.get("x");
		y = initial_state.get("y");
		psi = initial_state.get("psi");
		v = initial_state.get("v");
		a = 0.;

		for (int k = 1; k < N; k++) // For all timesteps
		{
			a = -deceleration;
			v = getEgoPrediction(k, "v") + a * k * dt;
			v = std::max(v, 0.);
			a = (v - getEgoPrediction(k, "v")) / (k * dt);

			x = initial_state.get("x") + v * k * dt * std::cos(initial_state.get("psi"));
			y = initial_state.get("y") + v * k * dt * std::sin(initial_state.get("psi"));

			setEgoPrediction(k, "x", x);
			setEgoPrediction(k, "y", y);
			setEgoPrediction(k, "psi", psi);
			setEgoPrediction(k, "v", v);
			setEgoPrediction(k, "a", a);
		}
	}

	void Solver::initializeWarmstart(const State &initial_state, bool shift_previous_solution_forward)
	{
		if (shift_previous_solution_forward)
		{
			/** @note warmstart shifting the previous output by one time step */
			// [initial_state, x_2, x_3, ..., x_N-1, x_N-1]
			for (int k = 0; k < N; k++) // For all timesteps
			{
				for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it) // For all inputs and states
				{
					if (k == 0) // Load the current state at k = 0
						setEgoPrediction(0, it->first.as<std::string>(), initial_state.get(it->first.as<std::string>()));
					else if (k == N - 1) // extrapolate with the terminal state at k = N-1
						setEgoPrediction(k, it->first.as<std::string>(), getOutput(k, it->first.as<std::string>()));
					else // use x_{k+1} to initialize x_{k} (note that both have the initial state)
						setEgoPrediction(k, it->first.as<std::string>(), getOutput(k + 1, it->first.as<std::string>()));
				}
			}
		}
		else
		{

			/** @note warmstart maintaining the previous output */
			// [initial_state, x_1, x_2, ..., x_N-1, x_N]
			for (int k = 0; k < N; k++) // For all timesteps
			{
				for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it) // For all inputs and states
				{
					if (k == 0) // Load the current state at k = 0
						setEgoPrediction(0, it->first.as<std::string>(), initial_state.get(it->first.as<std::string>()));
					else // use x_{k+1} to initialize x_{k} (note that both have the initial state)
						setEgoPrediction(k, it->first.as<std::string>(), getOutput(k, it->first.as<std::string>()));
				}
			}
		}
	}

	void Solver::loadWarmstart()
	{
		loadForcesWarmstart(_params, _output);
	}

	void Solver::setEgoPrediction(unsigned int k, std::string &&var_name, double value)
	{
		int index = _model_map[var_name][1].as<int>();
		_params.x0[k * nvar + index] = value;
	}

	double Solver::getEgoPrediction(unsigned int k, std::string &&var_name)
	{

		int index = _model_map[var_name][1].as<int>();
		return _params.x0[k * nvar + index];
	}

	void Solver::setEgoPredictionPosition(unsigned int k, const Eigen::Vector2d &value)
	{
		setEgoPrediction(k, "x", value(0));
		setEgoPrediction(k, "y", value(1));
	}

	Eigen::Vector2d Solver::getEgoPredictionPosition(unsigned int k)
	{
		return Eigen::Vector2d(getEgoPrediction(k, "x"), getEgoPrediction(k, "y"));
	}

	void Solver::setReinitialize(const bool reinitialize)
	{
		setForcesReinitialize(_params, reinitialize);
	}

	int Solver::solve()
	{
		int exit_code = Solver_solve(&_params, &_output, &_info, _solver_memory_handle, stdout, extfunc_eval_);
		return exit_code;
	}

	double Solver::getOutput(int k, std::string &&state_name) const
	{
		return getForcesOutput(_output, k, _model_map[state_name][1].as<int>());
	}

	std::string Solver::explainExitFlag(int exitflag)
	{
		switch (exitflag)
		{
		case -100:
			return "Invalid Forces Pro License (is the proxy running?)";
		case -6:
			return "Infeasible Optimization Problem";
		case -7:
			return "Infeasible Optimization Problem";
		case 2:
			return "Computation time exceeded allocated time";
		case 0:
			return "Exceeded maximum number of iterations";
		case 1:
			return "Successfully solved optimization problem";
		case -1:
			return "Guidance planner failed to find a solution";
		default:
			return "Unknown exit code";
		}
	}

	void Solver::printIfBoundLimited() const
	{
		// For all outputs, check whether they are close (within 1e-2) to their bounds on either side
		for (int k = 0; k < N; k++)
		{
			for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it)
			{
				if (k == 0 && it->second[0].as<std::string>() == "x")
					continue;

				if (std::abs(getOutput(k, it->first.as<std::string>()) - it->second[2].as<double>()) < 1e-2)
				{
					LOG_WARN_THROTTLE(500, it->first.as<std::string>() + " limited by lower bound");
				}
				if (std::abs(getOutput(k, it->first.as<std::string>()) - it->second[3].as<double>()) < 1e-2)
				{
					LOG_WARN_THROTTLE(500, it->first.as<std::string>() + " limited by upper bound");
				}
			}
		}
	}

	void Solver::printParameters(int k)
	{
		for (auto it = _parameter_map.begin(); it != _parameter_map.end(); ++it)
		{
			LOG_VALUE(it->first.as<std::string>(), getParameter(k, it->first.as<std::string>()));
		}
	}
} // namespace MPCPlanner