#include "mpc-planner-solver/solver_interface.h"

extern "C"
{
	Solver_extfunc extfunc_eval_ = &Solver_adtool2forces;
}

namespace MPCPlanner
{

	State::State()
	{
		loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "solver_settings"), _config);
		loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "model_map"), _model_map);
		initialize();
	}

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
		// LOG_INFO(nx << ", " << nu << ", " << N);
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

	void Solver::setVar(unsigned int k, std::string &&var_name, double value)
	{
		int index = _model_map[var_name][1].as<int>();
		_params.x0[k * nvar + index] = value;
	}

	double Solver::getVar(unsigned int k, std::string &&var_name)
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
		if (k == 0)
			return _output.x01[_model_map[state_name][1].as<int>()];
		if (k == 1)
			return _output.x02[_model_map[state_name][1].as<int>()];
		if (k == 2)
			return _output.x03[_model_map[state_name][1].as<int>()];
		if (k == 3)
			return _output.x04[_model_map[state_name][1].as<int>()];
		if (k == 4)
			return _output.x05[_model_map[state_name][1].as<int>()];
		if (k == 5)
			return _output.x06[_model_map[state_name][1].as<int>()];
		if (k == 6)
			return _output.x07[_model_map[state_name][1].as<int>()];
		if (k == 7)
			return _output.x08[_model_map[state_name][1].as<int>()];
		if (k == 8)
			return _output.x09[_model_map[state_name][1].as<int>()];
		if (k == 9)
			return _output.x10[_model_map[state_name][1].as<int>()];
		if (k == 10)
			return _output.x11[_model_map[state_name][1].as<int>()];
		if (k == 11)
			return _output.x12[_model_map[state_name][1].as<int>()];
		if (k == 12)
			return _output.x13[_model_map[state_name][1].as<int>()];
		if (k == 13)
			return _output.x14[_model_map[state_name][1].as<int>()];
		if (k == 14)
			return _output.x15[_model_map[state_name][1].as<int>()];
		if (k == 15)
			return _output.x16[_model_map[state_name][1].as<int>()];
		if (k == 16)
			return _output.x17[_model_map[state_name][1].as<int>()];
		if (k == 17)
			return _output.x18[_model_map[state_name][1].as<int>()];
		if (k == 18)
			return _output.x19[_model_map[state_name][1].as<int>()];
		if (k == 19)
			return _output.x20[_model_map[state_name][1].as<int>()];
		if (k == 20)
			return _output.x21[_model_map[state_name][1].as<int>()];
		if (k == 21)
			return _output.x22[_model_map[state_name][1].as<int>()];
		if (k == 22)
			return _output.x23[_model_map[state_name][1].as<int>()];
		if (k == 23)
			return _output.x24[_model_map[state_name][1].as<int>()];
		if (k == 24)
			return _output.x25[_model_map[state_name][1].as<int>()];
		if (k == 25)
			return _output.x26[_model_map[state_name][1].as<int>()];
		if (k == 26)
			return _output.x27[_model_map[state_name][1].as<int>()];
		if (k == 27)
			return _output.x28[_model_map[state_name][1].as<int>()];
		if (k == 28)
			return _output.x29[_model_map[state_name][1].as<int>()];
		if (k == 29)
			return _output.x30[_model_map[state_name][1].as<int>()];
		if (k == 30)
			return _output.x31[_model_map[state_name][1].as<int>()];
		if (k == 31)
			return _output.x32[_model_map[state_name][1].as<int>()];
		if (k == 32)
			return _output.x33[_model_map[state_name][1].as<int>()];
		if (k == 33)
			return _output.x34[_model_map[state_name][1].as<int>()];
		if (k == 34)
			return _output.x35[_model_map[state_name][1].as<int>()];
		if (k == 35)
			return _output.x36[_model_map[state_name][1].as<int>()];
		if (k == 36)
			return _output.x37[_model_map[state_name][1].as<int>()];
		if (k == 37)
			return _output.x38[_model_map[state_name][1].as<int>()];
		if (k == 38)
			return _output.x39[_model_map[state_name][1].as<int>()];
		if (k == 39)
			return _output.x40[_model_map[state_name][1].as<int>()];
	}
}; // namespace MPCPlanner