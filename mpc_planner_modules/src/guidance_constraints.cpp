#include "mpc_planner_modules/guidance_constraints.h"

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/data_visualization.h>

#include <ros_tools/visuals.h>
#include <ros_tools/profiling.h>

#include <omp.h>

namespace MPCPlanner
{
    GuidanceConstraints::LocalPlanner::LocalPlanner(int _id, bool _is_original_planner)
        : id(_id), is_original_planner(_is_original_planner)
    {
        local_solver = std::make_shared<Solver>(_id + 1); // Populate 1-n_solvers (leave 0 for the regular solver)
        guidance_constraints = std::make_unique<LinearizedConstraints>(local_solver);
        safety_constraints = std::make_unique<GUIDANCE_CONSTRAINTS_TYPE>(local_solver);

        guidance_constraints->setTopologyConstraints();
    }

    GuidanceConstraints::GuidanceConstraints(std::shared_ptr<Solver> solver)
        : ControllerModule(ModuleType::CONSTRAINT, solver, "guidance_constraints")
    {
        LOG_INITIALIZE("Guidance Constraints");

        global_guidance_ = std::make_unique<GuidancePlanner::GlobalGuidance>();
        // config_->N_pedestrians_ = GuidancePlanner::Config::N; // Increase the horizon expected for pedestrians to the PRM horizon

        // Initialize the constraint modules
        int n_solvers = global_guidance_->GetConfig()->n_paths_; // + 1 for the main lmpcc solver?
        LOG_VALUE("Solvers", n_solvers);
        for (int i = 0; i < n_solvers; i++)
        {
            planners_.emplace_back(i);
        }

        if (CONFIG["t-mpc"]["use_t-mpc++"].as<bool>()) // ADD IT AS FIRST PLAN
        {
            LOG_INFO("Using T-MPC++ (Adding the non-guided planner in parallel)");
            planners_.emplace_back(n_solvers, true);
        }

        LOG_INITIALIZED();
    }

    void GuidanceConstraints::update(State &state, const RealTimeData &data, ModuleData &module_data)
    {
        (void)data;
        (void)module_data;
        LOG_MARK("Guidance Constraints: Update");

        // Convert static obstacles
        if (!module_data.static_obstacles.empty())
        {
            std::vector<GuidancePlanner::Halfspace> halfspaces;
            for (size_t i = 0; i < module_data.static_obstacles[0].size(); i++)
            {
                halfspaces.emplace_back(module_data.static_obstacles[0][i].A, module_data.static_obstacles[0][i].b);
            }
            global_guidance_->LoadStaticObstacles(halfspaces); // Load static obstacles represented by halfspaces
        }

        if (CONFIG["t-mpc"]["use_t-mpc++"].as<bool>() && global_guidance_->GetConfig()->n_paths_ == 0) // No global guidance
            return;

        // Set the goals of the global guidance planner
        global_guidance_->SetStart(state.getPos(), state.get("psi"), state.get("v"));
        global_guidance_->SetReferenceVelocity(CONFIG["weights"]["reference_velocity"].as<double>());

        if (!CONFIG["enable_output"].as<bool>())
        {
            LOG_INFO_THROTTLE(15000, "Not propagating nodes (output is disabled)");
            global_guidance_->DoNotPropagateNodes();
        }
        /** @note Reference path */
        bool two_way = CONFIG["road"]["two_way"].as<bool>();
        double road_width_left_ = CONFIG["road"]["width"].as<double>() / 2.;
        double road_width_right_ = CONFIG["road"]["width"].as<double>() / 2.;

        /** @todo Find where we are on the spline */
        int current_segment;
        double current_s;
        _spline->findClosestPoint(state.getPos(), current_segment, current_s);

        double road_width_left = two_way ? road_width_left_ * 3. : road_width_left_;

        global_guidance_->LoadReferencePath(std::max(0., current_s), _spline,
                                            road_width_left - CONFIG["robot_radius"].as<double>() - 0.1,
                                            road_width_right_ - CONFIG["robot_radius"].as<double>() - 0.1);

        // global_guidance_->SetGoals({GuidancePlanner::Goal(state.getPos() + Eigen::Vector2d(4., 0.), 0.),
        //                             GuidancePlanner::Goal(state.getPos() + Eigen::Vector2d(4., 3.), 1.),
        //                             GuidancePlanner::Goal(state.getPos() + Eigen::Vector2d(4., -3.), 1.)});
        LOG_MARK("Running Guidance Search");
        global_guidance_->Update(); // data); /** @note The main update */
        // LOG_VALUE("Number of Guidance Trajectories", global_guidance_->NumberOfGuidanceTrajectories());
        empty_data_ = data;
        empty_data_.dynamic_obstacles.clear();
    }

    void GuidanceConstraints::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
    {
        (void)module_data;
        (void)data;
        if (k == 0)
        {
            // _solver->setTimeout(config_->solver_timeout_ / 1000.); // Limit the solver to 40 ms
            LOG_MARK("Guidance Constraints does not need to set parameters");
        }
    }

    int GuidanceConstraints::optimize(State &state, const RealTimeData &data, ModuleData &module_data)
    {
        // Required for parallel call to the solvers when using Forces
        omp_set_nested(1);
        omp_set_max_active_levels(2);
        omp_set_dynamic(0);
        LOG_MARK("Guidance Constraints: optimize");

        if (!CONFIG["t-mpc"]["use_t-mpc++"].as<bool>() && !global_guidance_->Succeeded())
            return 0;

#pragma omp parallel for num_threads(8)
        for (auto &planner : planners_)
        {
            PROFILE_SCOPE("Guidance Constraints: Parallel Optimization");
            planner.result.Reset();
            planner.disabled = false;

            if (planner.id >= global_guidance_->NumberOfGuidanceTrajectories()) // Only enable the solvers that are needed
            {
                if (!planner.is_original_planner) // We still want to add the original planner!
                {
                    planner.disabled = true;
                    continue;
                }
            }

            // Copy the data from the main solver
            auto &solver = planner.local_solver;
            LOG_MARK("Planner [" << planner.id << "]: Copying data from main solver");
            *solver = *_solver; // Copy the main solver

            // CONSTRUCT CONSTRAINTS
            if (planner.is_original_planner || (!CONFIG["t-mpc"]["enable_constraints"].as<bool>()))
            {
                planner.guidance_constraints->update(state, empty_data_, module_data);
                planner.safety_constraints->update(state, data, module_data); // Updates collision avoidance constraints
            }
            else
            {
                LOG_MARK("Planner [" << planner.id << "]: Loading guidance into the solver and constructing constraints");

                initializeSolverWithGuidance(planner);

                planner.guidance_constraints->update(state, data, module_data); // Updates linearization of constraints
                planner.safety_constraints->update(state, data, module_data);   // Updates collision avoidance constraints
            }

            // LOAD PARAMETERS
            LOG_MARK("Planner [" << planner.id << "]: Loading updated parameters into the solver");
            for (int k = 0; k < _solver->N; k++)
            {
                if (planner.is_original_planner)
                    planner.guidance_constraints->setParameters(empty_data_, module_data, k); // Set this solver's parameters
                else
                    planner.guidance_constraints->setParameters(data, module_data, k); // Set this solver's parameters

                planner.safety_constraints->setParameters(data, module_data, k);
            }

            /** @todo Set timeout */
            // planner.local_solver->_params.solver_timeout(1. / (config_->clock_frequency_) - (ros::Time::now() - data.control_loop_time_).seconds() - 0.005);
            planner.local_solver->_params.solver_timeout = 1. / CONFIG["control_frequency"].as<double>() - 20. / 1000.;

            // SOLVE OPTIMIZATION
            // if (enable_guidance_warmstart_)
            planner.local_solver->loadWarmstart();
            LOG_MARK("Planner [" << planner.id << "]: Solving ...");
            planner.result.exit_code = solver->solve();
            // solver_results_[i].exit_code =ellipsoidal_constraints_[solver->solver_id_].Optimize(solver.get()); // IF THIS OPTIMIZATION EXISTS!
            LOG_MARK("Planner [" << planner.id << "]: Done! (exitcode = " << planner.result.exit_code << ")");

            // ANALYSIS AND PROCESSING
            planner.result.success = planner.result.exit_code == 1;
            planner.result.objective = solver->_info.pobj; // How good is the solution?

            if (planner.is_original_planner) // We did not use any guidance!
            {
                planner.result.guidance_ID = 2 * global_guidance_->GetConfig()->n_paths_; // one higher than the maximum number of topology classes
                planner.result.color = -1;
            }
            else
            {
                auto &guidance_trajectory = global_guidance_->GetGuidanceTrajectory(planner.id); // planner.local_solver->_solver_id);
                planner.result.guidance_ID = guidance_trajectory.topology_class;                 // We were using this guidance
                planner.result.color = guidance_trajectory.color_;                               // A color index to visualize with

                if (guidance_trajectory.previously_selected_) // Prefer the selected trajectory
                    planner.result.objective *= 1. / global_guidance_->GetConfig()->selection_weight_consistency_;
            }
        }

        omp_set_dynamic(1);

        // DECISION MAKING
        best_planner_index_ = FindBestPlanner();
        if (best_planner_index_ == -1)
        {
            LOG_WARN_THROTTLE(500, "Failed to find a feasible trajectory in any of the " +
                                       std::to_string(planners_.size()) + " optimizations.");
            return planners_[0].result.exit_code;
        }

        auto &best_planner = planners_[best_planner_index_];
        auto &best_solver = best_planner.local_solver;
        // LOG_INFO("Best Planner ID: " << best_planner.id);

        // VISUALIZATION
        best_planner.guidance_constraints->visualize(data, module_data);
        best_planner.safety_constraints->visualize(data, module_data);

        // Communicate to the guidance which topology class we follow (none if it was the original planner)
        global_guidance_->OverrideSelectedTrajectory(best_planner.result.guidance_ID, best_planner.is_original_planner);

        _solver->_output = best_solver->_output; // Load the solution into the main lmpcc solver
        _solver->_info = best_solver->_info;
        _solver->_params = best_solver->_params;

        return best_planner.result.exit_code; // Return its exit code
    }

    void GuidanceConstraints::initializeSolverWithGuidance(LocalPlanner &planner)
    {
        auto &solver = planner.local_solver;

        // // Initialize the solver with the guidance trajectory
        // RosTools::CubicSpline2D<tk::spline> &trajectory_spline = global_guidance_->GetGuidanceTrajectory(solver->_solver_id).spline.GetTrajectory();
        RosTools::Spline2D &trajectory_spline = global_guidance_->GetGuidanceTrajectory(planner.id).spline.GetTrajectory();

        // Initialize the solver in the selected local optimum
        // I.e., set for each k, x(k), y(k) ...
        // The time indices are wrong here I think
        for (int k = 1; k < solver->N; k++) // note that the 0th velocity is the current velocity
        {
            // int index = k + 1;
            int index = k;
            Eigen::Vector2d cur_position = trajectory_spline.getPoint((double)(index)*solver->dt); // The plan is one ahead
            // global_guidance_->ProjectToFreeSpace(cur_position, k + 1);
            solver->setEgoPrediction(k, "x", cur_position(0));
            solver->setEgoPrediction(k, "y", cur_position(1));

            Eigen::Vector2d cur_velocity = trajectory_spline.getVelocity((double)(index)*solver->dt); // The plan is one ahead
            solver->setEgoPrediction(k, "psi", std::atan2(cur_velocity(1), cur_velocity(0)));
            solver->setEgoPrediction(k, "v", cur_velocity.norm());
        }
    }

    int GuidanceConstraints::FindBestPlanner()
    {
        // Find the best feasible solution
        double best_solution = 1e10;
        int best_index = -1;
        for (size_t i = 0; i < planners_.size(); i++)
        {
            auto &planner = planners_[i];
            if (planner.disabled) // Do not consider disabled planners
                continue;

            if (planner.result.success && planner.result.objective < best_solution)
            {
                best_solution = planner.result.objective;
                best_index = i;
            }
        }
        return best_index;
    }

    /** @brief Visualize the computations in this module  */
    void GuidanceConstraints::visualize(const RealTimeData &data, const ModuleData &module_data)
    {
        (void)data;
        (void)module_data;
        PROFILE_SCOPE("GuidanceConstraints::Visualize");
        LOG_MARK("Guidance Constraints: Visualize()");

        // global_guidance_->Visualize(highlight_selected_guidance_, visualized_guidance_trajectory_nr_);
        global_guidance_->Visualize(true, -1);
        for (size_t i = 0; i < planners_.size(); i++)
        {
            auto &planner = planners_[i];
            if (planner.disabled)
                continue;

            if (i == 0)
            {
                planner.guidance_constraints->visualize(data, module_data);
                planner.safety_constraints->visualize(data, module_data);
            }

            // Visualize the warmstart
            Trajectory initial_trajectory;
            for (int k = 1; k < planner.local_solver->N; k++)
                initial_trajectory.add(planner.local_solver->getEgoPrediction(k, "x"), planner.local_solver->getEgoPrediction(k, "y"));
            visualizeTrajectory(initial_trajectory, _name + "/warmstart_trajectories", false, 0.2, 20, 20);

            // Visualize the optimized trajectory
            if (planner.result.success)
            {
                Trajectory trajectory;
                for (int k = 1; k < _solver->N; k++)
                    trajectory.add(planner.local_solver->getOutput(k, "x"), planner.local_solver->getOutput(k, "y"));

                if ((int)i == best_planner_index_)
                    visualizeTrajectory(trajectory, _name + "/optimized_trajectories", false, 1.0, -1, 12, true, false);
                else if (planner.is_original_planner)
                    visualizeTrajectory(trajectory, _name + "/optimized_trajectories", false, 1.0, 11, 12, true, false);
                else
                    visualizeTrajectory(trajectory, _name + "/optimized_trajectories", false, 0.2, planner.result.color, global_guidance_->GetConfig()->n_paths_);
            }
        }

        {
            PROFILE_SCOPE("TEST");
            VISUALS.getPublisher(_name + "/optimized_trajectories").publish();
            VISUALS.getPublisher(_name + "/warmstart_trajectories").publish();
        }
    }

    bool GuidanceConstraints::isDataReady(const RealTimeData &data, std::string &missing_data)
    {
        (void)data;
        (void)missing_data;

        bool ready = true;
        ready = ready && planners_[0].guidance_constraints->isDataReady(data, missing_data);
        ready = ready && planners_[0].safety_constraints->isDataReady(data, missing_data);
        if (!ready)
            return false;

        if (_spline == nullptr)
        {
            missing_data += "Reference Path, ";
            return false;
        }

        return true;
    }

    //     /** @brief Load obstacles into the Homotopy module */
    void GuidanceConstraints::onDataReceived(RealTimeData &data, std::string &&data_name)
    {
        if (data_name == "reference_path") // New
        {
            LOG_MARK("Received Reference Path");

            _spline = std::make_unique<RosTools::Spline2D>(data.reference_path.x, data.reference_path.y); // Construct a spline from the given points
        }
        else if (data_name == "goal") // New
        {
            LOG_WARN("Received Goal");

            std::vector<double> x = {data.goal(0), data.goal(0) + 1., data.goal(0) + 1., data.goal(0)};
            std::vector<double> y = {data.goal(1), data.goal(1), data.goal(1) + 1., data.goal(1) + 1.};

            _spline = std::make_unique<RosTools::Spline2D>(x, y); // Construct a spline from the given points
        }

        // We wait for both the obstacles and state to arrive before we compute here
        if (data_name == "dynamic obstacles")
        {
            LOG_MARK("Guidance Constraints: Received dynamic obstacles");

            // #pragma omp parallel for num_threads(8)
            for (auto &planner : planners_)
            {
                planner.safety_constraints->onDataReceived(data, std::forward<std::string>(data_name));
            }

            std::vector<GuidancePlanner::Obstacle> obstacles;
            for (auto &obstacle : data.dynamic_obstacles)
            {
                std::vector<Eigen::Vector2d> positions;
                positions.push_back(obstacle.position); /** @note Strange that we need k = 0 here */
                for (size_t k = 0; k < std::max(obstacle.prediction.modes[0].size(), (size_t)GuidancePlanner::Config::N); k++)
                {
                    positions.push_back(obstacle.prediction.modes[0][k].position);
                }
                obstacles.emplace_back(obstacle.index, positions, obstacle.radius + data.robot_area[0].radius);
            }

            global_guidance_->LoadObstacles(obstacles, {});
        }
    }

    void GuidanceConstraints::reset()
    {
        _spline.reset(nullptr);
        global_guidance_->Reset();

        for (auto &planner : planners_)
            planner.local_solver->reset();
    }

    //     // void GuidanceConstraints::ReconfigureCallback(SolverInterface *solver_interface, lmpcc::PredictiveControllerConfig &config,
    //     //                                               uint32_t level, bool first_callback)
    //     // {
    //     //   if (first_callback)
    //     //   {
    //     //     config.spline_consistency = global_guidance_->GetConfig()->selection_weight_consistency_;
    //     //   }
    //     //   else
    //     //   {
    //     //     global_guidance_->GetConfig()->selection_weight_consistency_ = config.spline_consistency;
    //     //   }

    //     //   global_guidance_->SetReferenceVelocity(config.velocity_reference);
    //     //   config_->visualized_guidance_trajectory_nr_ = config.visualize_trajectory_nr;
    //     //   config_->highlight_selected_guidance_ = config.highlight_selected;
    //     // }

    //     void GuidanceConstraints::ExportData(RosTools::DataSaver &data_saver)
    //     {
    //         data_saver.AddData("runtime_guidance", global_guidance_->GetLastRuntime());
    //         for (size_t i = 0; i < planners_.size(); i++) // auto &solver : solvers_)
    //         {
    //             auto &planner = planners_[i];
    //             if (planner.result.success)
    //                 data_saver.AddData("objective_" + std::to_string(i), planner.local_solver->forces_info_.pobj);
    //             else
    //                 data_saver.AddData("objective_" + std::to_string(i), -1.);

    //             if (planner.is_original_planner)
    //             {
    //                 data_saver.AddData("original_planner_id", planner.id); // To identify which one is the original planner
    //             }
    //             auto &vehicle_regions = planner.local_solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
    //             for (size_t k = 0; k < vehicle_regions.size(); k++)
    //                 data_saver.AddData("solver" + std::to_string(i) + "_plan" + std::to_string(k), vehicle_regions[k].discs_[0].AsVector2d());

    //             data_saver.AddData("active_constraints_" + std::to_string(planner.id), planner.guidance_constraints->NumActiveConstraints(planner.local_solver.get()));
    //         }

    //         data_saver.AddData("best_planner_idx", best_planner_index_);
    //         if (best_planner_index_ != -1)
    //             data_saver.AddData("gmpcc_objective", planners_[best_planner_index_].local_solver->forces_info_.pobj);
    //         else
    //             data_saver.AddData("gmpcc_objective", -1);

    //         global_guidance_->ExportData(data_saver);
    //     }
    // }
}