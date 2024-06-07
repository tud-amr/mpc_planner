#include <mpc_planner_solver/acados_solver_interface.h>

#include <mpc_planner_util/parameters.h>

namespace MPCPlanner
{
    Solver::Solver(int solver_id)
    {
        _solver_id = solver_id;

        loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "solver_settings"), _config);
        loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "parameter_map"), _parameter_map);
        loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "model_map"), _model_map);

        _acados_ocp_capsule = Solver_acados_create_capsule();

        // there is an opportunity to change the number of shooting intervals in C without new code generation
        N = SOLVER_N;
        nu = _config["nu"].as<unsigned int>();
        nx = _config["nx"].as<unsigned int>();
        nvar = _config["nvar"].as<unsigned int>();
        npar = _config["npar"].as<unsigned int>();
        dt = CONFIG["integrator_step"].as<double>();

        _num_iterations = CONFIG["solver_settings"]["acados"]["iterations"].as<int>();
        if (CONFIG["solver_settings"]["acados"]["solver_type"].as<std::string>() == "SQP")
            _num_iterations = 1;

        // allocate the array and fill it accordingly
        double *new_time_steps = NULL;
        int status = Solver_acados_create_with_discretization(_acados_ocp_capsule, N, new_time_steps);

        if (status)
        {
            printf("Solver_acados_create() returned status %d. Exiting.\n", status);
            exit(1);
        }

        _nlp_config = Solver_acados_get_nlp_config(_acados_ocp_capsule);
        _nlp_dims = Solver_acados_get_nlp_dims(_acados_ocp_capsule);
        _nlp_in = Solver_acados_get_nlp_in(_acados_ocp_capsule);
        _nlp_out = Solver_acados_get_nlp_out(_acados_ocp_capsule);
        _nlp_solver = Solver_acados_get_nlp_solver(_acados_ocp_capsule);
        _nlp_opts = Solver_acados_get_nlp_opts(_acados_ocp_capsule);

        reset();
    }

    Solver::~Solver()
    {
        // free solver
        int status = Solver_acados_free(_acados_ocp_capsule);
        if (status)
        {
            LOG_INFO("Solver_acados_free() returned status " << status);
        }
        // free solver capsule
        status = Solver_acados_free_capsule(_acados_ocp_capsule);
        if (status)
        {
            LOG_INFO("Solver_acados_free_capsule() returned status " << status);
        }
    }

    Solver &Solver::operator=(const Solver &rhs)
    {
        _params = rhs._params;
        _output = rhs._output;

        return *this;
    }

    void Solver::reset()
    {
        _params = AcadosParameters();
        _info = AcadosInfo();
        _output = AcadosOutput();
    }

    int Solver::solve()
    {
        int status = 1;

        // _params.printParameters(_parameter_map);

        // Set initial state
        ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "idxbx", _params.getIdxbx0());
        ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "lbx", _params.xinit);
        ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "ubx", _params.xinit);

        // Set parameters
        for (int k = 0; k <= N; k++)
        {
            if (k == N)
                Solver_acados_update_params(_acados_ocp_capsule, k, &_params.all_parameters[(N - 1) * SOLVER_NP], SOLVER_NP); // Insert the second to last set of parameters
            else if (k == 0)
                Solver_acados_update_params(_acados_ocp_capsule, k, &_params.all_parameters[SOLVER_NP], SOLVER_NP); // Insert the second to last set of parameters
            else
                Solver_acados_update_params(_acados_ocp_capsule, k, &_params.all_parameters[k * SOLVER_NP], SOLVER_NP);
        }

        _info = AcadosInfo();

        // solve ocp in loop
        int rti_phase = 0;

        // bool warmstart_qp = true;
        // ocp_nlp_solver_opts_set(_nlp_config, _nlp_opts, "warm_start_first_qp", &warmstart_qp);
        // ocp_nlp_solver_opts_update(_nlp_config, _nlp_dims, _nlp_opts);
        ocp_nlp_solver_reset_qp_memory(_nlp_solver, _nlp_in, _nlp_out);
        ocp_nlp_precompute(_nlp_solver, _nlp_in, _nlp_out);
        for (int iteration = 0; iteration < _num_iterations; iteration++)
        {
            ocp_nlp_solver_opts_set(_nlp_config, _nlp_opts, "rti_phase", &rti_phase);
            status = Solver_acados_solve(_acados_ocp_capsule);

            ocp_nlp_get(_nlp_config, _nlp_solver, "time_tot", &_info.elapsed_time);
            _info.solvetime += _info.elapsed_time;
            _info.min_time = MIN(_info.elapsed_time, _info.min_time);
        }

        ocp_nlp_get(_nlp_config, _nlp_solver, "nlp_res", &_info.nlp_res);

        // Get output
        for (int k = 0; k <= _nlp_dims->N; k++)
            ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, k, "x", &_output.xtraj[k * NX]);
        for (int k = 0; k < _nlp_dims->N; k++)
            ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, k, "u", &_output.utraj[k * NU]);
        // _output.print();

        if (status == ACADOS_SUCCESS)
        {
            LOG_MARK("Solver_acados_solve(): SUCCESS!");
        }
        else
        {
            LOG_MARK("Solver_acados_solve() failed with status " << status);
        }

        // Get INFO
        ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, 0, "kkt_norm_inf", &_info.kkt_norm_inf);
        ocp_nlp_get(_nlp_config, _nlp_solver, "sqp_iter", &_info.sqp_iter);

        // Solver_acados_print_stats(_acados_ocp_capsule);
        // _info.print();

        // Map to FORCES output
        if (status == 0 || status == 2) // Success (max iterations = success for now)
            status = 1;
        else if (status == 1)
            status = 0;

        return status;
    }

    // PARAMETERS //
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

    // XINIT //

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

    // WARMSTART //

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

    void Solver::loadWarmstart()
    {
        // Load from x0 into traj_x and traj_u
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, i, "x", &_params.x0[nvar * i + nu]);
            ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, i, "u", &_params.x0[nvar * i]);
        }

        ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, N, "x", &_params.x0[nvar * N + nu]);
    }

    void Solver::initializeWithState(const State &initial_state)
    {
        for (int k = 0; k <= N; k++) // For all timesteps
        {
            for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it) // For all inputs and states
            {
                setEgoPrediction(k, it->first.as<std::string>(), initial_state.get(it->first.as<std::string>()));
            }
        }
    }

    void Solver::initializeWithBraking(const State &initial_state)
    {
        LOG_MARK("Initialize Plan with a Braking Plan");
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
            for (int k = 0; k <= N; k++) // For all timesteps
            {
                for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it) // For all inputs and states
                {
                    if (k == 0) // Load the current state at k = 0
                        setEgoPrediction(0, it->first.as<std::string>(), initial_state.get(it->first.as<std::string>()));
                    else if (k == N - 1) // extrapolate with the terminal state at k = N-1
                        setEgoPrediction(N - 1, it->first.as<std::string>(), getOutput(N - 1, it->first.as<std::string>()));
                    else if (k == N)
                        setEgoPrediction(N, it->first.as<std::string>(), getOutput(N - 1, it->first.as<std::string>()));
                    else // use x_{k+1} to initialize x_{k} (note that both have the initial state)
                        setEgoPrediction(k, it->first.as<std::string>(), getOutput(k + 1, it->first.as<std::string>()));
                }
            }
        }
        else
        {

            /** @note warmstart maintaining the previous output */
            // [initial_state, x_1, x_2, ..., x_N-1, x_N]
            for (int k = 0; k <= N; k++) // For all timesteps
            {
                for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it) // For all inputs and states
                {
                    if (k == 0) // Load the current state at k = 0
                        setEgoPrediction(0, it->first.as<std::string>(), initial_state.get(it->first.as<std::string>()));
                    else if (k == N)
                        setEgoPrediction(N, it->first.as<std::string>(), getOutput(N - 1, it->first.as<std::string>()));
                    else // use x_{k+1} to initialize x_{k} (note that both have the initial state)
                        setEgoPrediction(k, it->first.as<std::string>(), getOutput(k, it->first.as<std::string>()));
                }
            }
        }
    }

    // OUTPUT //
    double Solver::getOutput(int k, std::string &&state_name) const
    {
        if (_model_map[state_name][0].as<std::string>() == "x")
        {
            return _output.xtraj[k * nx + _model_map[state_name][1].as<int>() - nu];
        }
        else
        {
            return _output.utraj[k * nu + _model_map[state_name][1].as<int>()];
        }
    }

    std::string Solver::explainExitFlag(int exitflag) const
    {
        switch (exitflag)
        {
        case 1: // We swap 0 and 1 for consistency with Forces Pro
            return "Success";
        case 0:
            return "Failure (no more information)";
        case 2:
            return "Failure (maximum number of iterations reached)";
        case 3:
            return "Failure (minimum step size reached)";
        case 4:
            return "Failure (QP Failure)";
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
}