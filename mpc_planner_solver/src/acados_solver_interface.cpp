#include <mpc_planner_solver/acados_solver_interface.h>

#include <mpc_planner_util/parameters.h>

#include <ros_tools/profiling.h>

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
        ocp_nlp_solver_reset_qp_memory(_nlp_solver, _nlp_in, _nlp_out);

        // _output = rhs._output;
        // *_acados_ocp_capsule = *rhs._acados_ocp_capsule;
        // Solver_acados_reset(_acados_ocp_capsule, 0);

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

        RosTools::Benchmarker iteration_timer("iteration");
        RosTools::Timer timeout_timer(_params.solver_timeout);
        timeout_timer.start();

        // _params.printParameters(_parameter_map);

        initializeOneIteration();
        double iteration_time = 0.;

        for (int iteration = 0; iteration < _num_iterations; iteration++)
        {
            iteration_timer.start();

            solveOneIteration();

            if (status != ACADOS_SUCCESS && _info.qp_status != 0)
                break;

            iteration_time += iteration_timer.stop();
            double avg_iteration_time = iteration_time / ((double)(iteration + 1));

            // Stop iterating if we ran out of time
            if (timeout_timer.currentDuration() + avg_iteration_time >= _params.solver_timeout)
            {
                LOG_WARN_THROTTLE(15000., "Timeout is enabled. Stopping after " << iteration + 1 << " iterations because planning time is exceeded");
                break;
            }
        }
        return completeOneIteration();
    }

    void Solver::initializeOneIteration()
    {
        // Set initial state
        ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "lbx", _params.xinit);
        ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "ubx", _params.xinit);

        // Set parameters
        for (int k = 0; k <= N; k++)
        {
            if (k == N)
                Solver_acados_update_params(_acados_ocp_capsule, k, &_params.all_parameters[(N - 1) * SOLVER_NP], SOLVER_NP); // Insert the second to last set of parameters
            else
                Solver_acados_update_params(_acados_ocp_capsule, k, &_params.all_parameters[k * SOLVER_NP], SOLVER_NP);
        }

        _info = AcadosInfo();

        // solve ocp in loop
        int rti_phase = 0; // 1 = prep, 2 = feedback, 0 = both

        ocp_nlp_solver_opts_set(_nlp_config, _nlp_opts, "rti_phase", &rti_phase);
        ocp_nlp_precompute(_nlp_solver, _nlp_in, _nlp_out);
    }

    int Solver::solveOneIteration()
    {
        int status = -1;

        status = Solver_acados_solve(_acados_ocp_capsule);

        ocp_nlp_get(_nlp_solver, "time_tot", &_info.elapsed_time);
        _info.solvetime += _info.elapsed_time;
        _info.min_time = MIN(_info.elapsed_time, _info.min_time);

        ocp_nlp_get(_nlp_solver, "qp_status", &_info.qp_status);

        _exit_code_one_iter = status;

        return status;
    }

    int Solver::completeOneIteration()
    {
        ocp_nlp_get(_nlp_solver, "nlp_res", &_info.nlp_res);

        // Compute and retrieve the cost
        ocp_nlp_eval_cost(_nlp_solver, _nlp_in, _nlp_out);
        ocp_nlp_get(_nlp_solver, "cost_value", &_info.pobj);

        // Get output
        for (int k = 0; k <= _nlp_dims->N; k++)
            ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, k, "x", &_output.xtraj[k * nx]);
        for (int k = 0; k < _nlp_dims->N; k++)
            ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, k, "u", &_output.utraj[k * nu]);

        double res_stat, res_eq, res_ineq, res_comp;
        ocp_nlp_get(_nlp_solver, "res_eq", &res_eq);
        if (res_eq > 1e-2 && _exit_code_one_iter == ACADOS_SUCCESS)
        {
            _exit_code_one_iter = ACADOS_QP_FAILURE;
        }

        if (_exit_code_one_iter == ACADOS_SUCCESS)
        {
            LOG_MARK("Solver_acados_solve(): SUCCESS!");
        }
        else
        {
            Solver_acados_reset(_acados_ocp_capsule, 1);
            ocp_nlp_solver_reset_qp_memory(_nlp_solver, _nlp_in, _nlp_out);
        }

        // Get INFO
        ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, 0, "kkt_norm_inf", &_info.kkt_norm_inf);
        ocp_nlp_get(_nlp_solver, "sqp_iter", &_info.sqp_iter);

        // Map to FORCES output
        if (_exit_code_one_iter == ACADOS_SUCCESS) // Success
            _exit_code_one_iter = 1;
        else if (_exit_code_one_iter == 1)
            _exit_code_one_iter = 0;

        return _exit_code_one_iter;
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
        for (int k = 0; k < N; k++)
        {
            ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, k, "x", &_params.x0[nvar * k + nu]);
            ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, k, "u", &_params.x0[nvar * k]);
        }

        ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, N, "x", &_params.x0[nvar * N + nu]);
    }

    void Solver::initializeWithState(const State &initial_state)
    {
        for (int k = 0; k <= N; k++) // For all timesteps
        {
            // LOG_HEADER(k);
            for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it) // For all inputs and states
            {
                if (_model_map[it->first.as<std::string>()][0].as<std::string>() == "x") // Set states to initial state
                    setEgoPrediction(k, it->first.as<std::string>(), initial_state.get(it->first.as<std::string>()));
                else
                    setEgoPrediction(k, it->first.as<std::string>(), 0.);

                // LOG_VALUE(it->first.as<std::string>(), getEgoPrediction(k, it->first.as<std::string>()));
            }
        }
    }

    void Solver::initializeWithBraking(const State &initial_state)
    {
        LOG_MARK("Initialize Plan with a Braking Plan");
        initializeWithState(initial_state); // Initialize all variables

        double x, y, psi, v, a, spline;
        double deceleration = std::abs(CONFIG["deceleration_at_infeasible"].as<double>());

        x = initial_state.get("x");
        y = initial_state.get("y");
        psi = initial_state.get("psi");
        v = initial_state.get("v");
        spline = initial_state.get("spline");
        a = -deceleration;

        setEgoPrediction(0, "x", x);
        setEgoPrediction(0, "y", y);
        setEgoPrediction(0, "psi", psi);
        setEgoPrediction(0, "v", v);
        setEgoPrediction(0, "spline", spline);
        setEgoPrediction(0, "a", a);
        setEgoPrediction(0, "w", 0);

        for (int k = 1; k <= N; k++) // For all timesteps
        {
            x += v * dt * std::cos(psi);
            y += v * dt * std::sin(psi);
            spline += v * dt;
            v += a * dt;
            v = std::max(v, 0.);

            setEgoPrediction(k, "x", x);
            setEgoPrediction(k, "y", y);
            setEgoPrediction(k, "psi", psi);
            setEgoPrediction(k, "v", v);
            setEgoPrediction(k, "spline", spline);
            setEgoPrediction(k, "a", a);
            setEgoPrediction(k, "w", 0);
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
            for (int k = 0; k < N; k++) // For all timesteps
            {
                for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it)                 // For all inputs and states
                    setEgoPrediction(k, it->first.as<std::string>(), getOutput(k, it->first.as<std::string>())); // Initialize with the previous output
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
            break;
        default:
            return "Unknown exit code";
        }

        switch (_info.qp_status)
        {
        case 1:
            return "QP Failure: No more information on QP failure";
        case 2:
            return "QP Failure: Max Iterations";
        case 3:
            return "QP Failure: Minimal Step Reached";
        case 4:
            return "QP Failure: NAN in solution";
        case 5:
            return "QP Failure: Inconsistent Equality Constraints";
        default:
            return "QP Failure: UNKNOWN";
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