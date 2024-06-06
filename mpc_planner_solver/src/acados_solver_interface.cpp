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

    void Solver::reset()
    {
        _params = AcadosParameters();
        _info = AcadosInfo();
        _output = AcadosOutput();
    }

    int Solver::solve()
    {
        int status;

        // _params.printParameters();

        // Set initial state
        ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "idxbx", _params.getIdxbx0());
        ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "lbx", _params.xinit);
        ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "ubx", _params.xinit);

        // Set parameters
        for (int ii = 0; ii <= N; ii++)
        {
            Solver_acados_update_params(_acados_ocp_capsule, ii, &_params.all_parameters[ii * NP], NP);
        }

        _info = AcadosInfo();

        // solve ocp in loop
        int rti_phase = 0;

        for (int ii = 0; ii < _info.NTIMINGS; ii++)
        {
            ocp_nlp_solver_opts_set(_nlp_config, _nlp_opts, "rti_phase", &rti_phase);

            status = Solver_acados_solve(_acados_ocp_capsule);

            ocp_nlp_get(_nlp_config, _nlp_solver, "time_tot", &_info.elapsed_time);
            _info.min_time = MIN(_info.elapsed_time, _info.min_time);
        }

        /* print solution and statistics */
        for (int ii = 0; ii <= _nlp_dims->N; ii++)
            ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, ii, "x", &_output.xtraj[ii * NX]);
        for (int ii = 0; ii < _nlp_dims->N; ii++)
            ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, ii, "u", &_output.utraj[ii * NU]);

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
        if (status == 0) // Success
            status = 1;
        else
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
        case 0:
            return "Success";
        case 1:
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
    /*
    void setSolverParameterAcceleration(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 0] = value;
    }
    void setSolverParameterAngularVelocity(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 1] = value;
    }
    void setSolverParameterContour(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 2] = value;
    }
    void setSolverParameterReferenceVelocity(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 3] = value;
    }
    void setSolverParameterVelocity(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 4] = value;
    }
    void setSolverParameterLag(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 5] = value;
    }
    void setSolverParameterTerminalAngle(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 6] = value;
    }
    void setSolverParameterTerminalContouring(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 7] = value;
    }
    void setSolverParameterSplineXA(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 8] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 17] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 26] = value;
    }
    void setSolverParameterSplineXB(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 9] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 18] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 27] = value;
    }
    void setSolverParameterSplineXC(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 10] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 19] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 28] = value;
    }
    void setSolverParameterSplineXD(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 11] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 20] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 29] = value;
    }
    void setSolverParameterSplineYA(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 12] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 21] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 30] = value;
    }
    void setSolverParameterSplineYB(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 13] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 22] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 31] = value;
    }
    void setSolverParameterSplineYC(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 14] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 23] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 32] = value;
    }
    void setSolverParameterSplineYD(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 15] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 24] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 33] = value;
    }
    void setSolverParameterSplineStart(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 16] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 25] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 34] = value;
    }
    void setSolverParameterSplineVA(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 35] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 39] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 43] = value;
    }
    void setSolverParameterSplineVB(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 36] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 40] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 44] = value;
    }
    void setSolverParameterSplineVC(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 37] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 41] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 45] = value;
    }
    void setSolverParameterSplineVD(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 38] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 42] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 46] = value;
    }
    void setSolverParameterLinConstraintA1(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 47] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 50] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 53] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 56] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 59] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 62] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 65] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 68] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 71] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 74] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 77] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 80] = value;
    }
    void setSolverParameterLinConstraintA2(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 48] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 51] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 54] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 57] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 60] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 63] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 66] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 69] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 72] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 75] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 78] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 81] = value;
    }
    void setSolverParameterLinConstraintB(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 49] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 52] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 55] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 58] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 61] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 64] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 67] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 70] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 73] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 76] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 79] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 82] = value;
    }
    void setSolverParameterEgoDiscRadius(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 83] = value;
    }
    void setSolverParameterEgoDiscOffset(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        (void)index;
        params.all_parameters[k * 169 + 84] = value;
    }
    void setSolverParameterEllipsoidObstX(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 85] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 92] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 99] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 106] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 113] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 120] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 127] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 134] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 141] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 148] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 155] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 162] = value;
    }
    void setSolverParameterEllipsoidObstY(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 86] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 93] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 100] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 107] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 114] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 121] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 128] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 135] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 142] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 149] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 156] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 163] = value;
    }
    void setSolverParameterEllipsoidObstPsi(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 87] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 94] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 101] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 108] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 115] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 122] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 129] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 136] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 143] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 150] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 157] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 164] = value;
    }
    void setSolverParameterEllipsoidObstMajor(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 88] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 95] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 102] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 109] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 116] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 123] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 130] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 137] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 144] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 151] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 158] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 165] = value;
    }
    void setSolverParameterEllipsoidObstMinor(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 89] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 96] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 103] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 110] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 117] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 124] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 131] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 138] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 145] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 152] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 159] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 166] = value;
    }
    void setSolverParameterEllipsoidObstChi(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 90] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 97] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 104] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 111] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 118] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 125] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 132] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 139] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 146] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 153] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 160] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 167] = value;
    }
    void setSolverParameterEllipsoidObstR(int k, Solver::AcadosParameters &params, const double value, int index)
    {
        if (index == 0)
            params.all_parameters[k * 169 + 91] = value;
        else if (index == 1)
            params.all_parameters[k * 169 + 98] = value;
        else if (index == 2)
            params.all_parameters[k * 169 + 105] = value;
        else if (index == 3)
            params.all_parameters[k * 169 + 112] = value;
        else if (index == 4)
            params.all_parameters[k * 169 + 119] = value;
        else if (index == 5)
            params.all_parameters[k * 169 + 126] = value;
        else if (index == 6)
            params.all_parameters[k * 169 + 133] = value;
        else if (index == 7)
            params.all_parameters[k * 169 + 140] = value;
        else if (index == 8)
            params.all_parameters[k * 169 + 147] = value;
        else if (index == 9)
            params.all_parameters[k * 169 + 154] = value;
        else if (index == 10)
            params.all_parameters[k * 169 + 161] = value;
        else if (index == 11)
            params.all_parameters[k * 169 + 168] = value;
    }*/
}