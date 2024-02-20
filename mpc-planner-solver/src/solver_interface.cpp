#include "mpc-planner-solver/solver_interface.h"

#include <mpc-planner-util/logging.h>

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

	void Solver::setParameter(int k, std::string &&parameter, double value)
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

	void Solver::setWarmstart(const State &initial_state)
	{
		for (int k = 0; k < N; k++) // For all timesteps
		{
			for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it) // For all inputs and states
			{
				if (k == 0)
					setEgoPrediction(0, it->first.as<std::string>(), initial_state.get(it->first.as<std::string>())); // Load the current state at k = 0
				else if (k == N - 1)
					setEgoPrediction(k, it->first.as<std::string>(), getOutput(k, it->first.as<std::string>())); // extrapolate with the terminal state at k = N-1
				else
					setEgoPrediction(k, it->first.as<std::string>(), getOutput(k + 1, it->first.as<std::string>())); // use x_{k+1} to initialize x_{k} (note that both have the initial state)
			}
		}
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

	int Solver::solve()
	{
		int exit_code = Solver_solve(&_params, &_output, &_info, _solver_memory_handle, stdout, extfunc_eval_);
		return exit_code;
	}

	double Solver::getOutput(int k, std::string &&state_name)
	{
		return getForcesOutput(_output, k, _model_map[state_name][1].as<int>());
	}
} // namespace MPCPlanner