#include "mpc-planner-modules/guidance_constraints.h"

#include <mpc-planner-util/parameters.h>
#include <mpc-planner-util/visuals.h>

#include <omp.h>

namespace MPCPlanner
{
    GuidanceConstraints::LocalPlanner::LocalPlanner(int _id, bool _is_original_planner)
        : id(_id), is_original_planner(_is_original_planner)
    {
        local_solver = std::make_shared<Solver>(_id + 1); // Populate 1-n_solvers (leave 0 for the regular solver)
        guidance_constraints = std::make_unique<LinearizedConstraints>(local_solver);
        safety_constraints = std::make_unique<GUIDANCE_CONSTRAINTS_TYPE>(local_solver);
    }

    GuidanceConstraints::GuidanceConstraints(std::shared_ptr<Solver> solver)
        : ControllerModule(ModuleType::CONSTRAINT, solver, "guidance_constraints")
    {
        LOG_INFO("Initializing Guidance Constraints Module");

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

        LOG_INFO("Guidance Constraints Initialized");
    }

    void GuidanceConstraints::update(State &state, const RealTimeData &data)
    {
        LOG_DEBUG("Guidance Constraints: Update");

        // First run the base class update to update our progress along the path
        // Contouring::Update(solver_interface, data);
        /** @todo Find where we are on the spline */

        // global_guidance_->LoadHalfspaces(); // Load static obstacles represented by halfspaces

        // Is this necessary?
        // planners_[0].guidance_constraints->Update(solver_interface, data);
        // planners_[0].safety_constraints->Update(solver_interface, data);

        if (CONFIG["t-mpc"]["use_t-mpc++"].as<bool>() && global_guidance_->GetConfig()->n_paths_ == 0) // No global guidance
            return;

        // Set the goals of the global guidance planner
        global_guidance_->SetStart(state.getPos(), state.get("psi"), state.get("v"));
        global_guidance_->SetReferenceVelocity(CONFIG["reference_velocity"].as<double>()); // solver_interface->weights_.velocity_reference_);

        /** @note Reference path */
        // Temporary
        bool enable_two_way_road_ = false;
        double road_width_left_ = 3.5;
        double road_width_right_ = 3.5;

        int current_segment;
        double current_s;
        _spline->findClosestPoint(state.getPos(), current_segment, current_s);

        double road_width_left = enable_two_way_road_ ? road_width_left_ * 3. : road_width_left_;
        _guidance_spline = std::make_unique<RosTools::CubicSpline2D<tk::spline>>(
            _spline->getXSpline(),
            _spline->getYSpline()); // Construct a spline from the given points

        global_guidance_->LoadReferencePath(std::max(0., current_s), _guidance_spline,
                                            road_width_left - CONFIG["robot_radius"].as<double>() - 0.1,
                                            road_width_right_ - CONFIG["robot_radius"].as<double>() - 0.1);

        // global_guidance_->SetGoals({GuidancePlanner::Goal(state.getPos() + Eigen::Vector2d(4., 0.), 0.),
        //                             GuidancePlanner::Goal(state.getPos() + Eigen::Vector2d(4., 3.), 1.),
        //                             GuidancePlanner::Goal(state.getPos() + Eigen::Vector2d(4., -3.), 1.)});
        LOG_DEBUG("Running Guidance Search");
        global_guidance_->Update(); // data); /** @note The main update */
        // LOG_VALUE("Number of Guidance Trajectories", global_guidance_->NumberOfGuidanceTrajectories());

        // Initialize the solver with the best (= 0) guidance trajectory, if configured
        // if (config_->enable_guidance_)
        // InitializeSolverWithGuidance(solver_interface);
    }

    void GuidanceConstraints::setParameters(const RealTimeData &data, int k)
    {
        if (k == 0)
        {
            // _solver->setTimeout(config_->solver_timeout_ / 1000.); // Limit the solver to 40 ms
            LOG_DEBUG("Guidance Constraints does not need to set parameters");
        }
    }

    int GuidanceConstraints::optimize(State &state, const RealTimeData &data)
    {
        // Required for parallel call to the solvers when using Forces
        omp_set_nested(1);
        omp_set_max_active_levels(2);
        omp_set_dynamic(0);
        LOG_DEBUG("Guidance Constraints: optimize");

        if (!CONFIG["t-mpc"]["use_t-mpc++"].as<bool>() && !global_guidance_->Succeeded())
            return 0;

        // #pragma omp parallel for num_threads(8)
        for (auto &planner : planners_)
        {
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
            LOG_DEBUG("Planner [" << planner.id << "]: Copying data from main solver");
            *solver = *_solver; // Copy the main solver

            // CONSTRUCT CONSTRAINTS
            if (planner.is_original_planner || (!CONFIG["t-mpc"]["enable_constraints"].as<bool>()))
            {
                // For the original problem we construct dummy constraints but with the static constraints (no guidance!)
                // LOG_INFO("Planner [" << planner.id << "]: Constructing dummy constraints");
                // empty_data_.halfspaces_ = data_ptr_->halfspaces_; // Copy in the halfspace data

                // solver->LoadInitialVehiclePredictions(); Note: we do not care about the initial position
                // solver->LoadVehiclePredictionsToInitialPlan
                planner.guidance_constraints->update(state, empty_data_);
                planner.safety_constraints->update(state, data); // Updates collision avoidance constraints
            }
            else
            {
                LOG_DEBUG("Planner [" << planner.id << "]: Loading guidance into the solver and constructing constraints");

                initializeSolverWithGuidance(planner);
                // solver->LoadInitialVehiclePredictions(); // Load vehicle predictions for constraint construction

                planner.guidance_constraints->update(state, data); // Updates linearization of constraints
                planner.safety_constraints->update(state, data);   // Updates collision avoidance constraints
            }

            // LOAD PARAMETERS
            // if (enable_guidance_warmstart_)
            // solver->loadInitialPlanAsWarmStart(); // Warmstart if configured

            LOG_DEBUG("Planner [" << planner.id << "]: Loading updated parameters into the solver");
            for (int k = 0; k < _solver->N; k++)
            {
                planner.guidance_constraints->setParameters(data, k); // Set this solver's parameters
                planner.safety_constraints->setParameters(data, k);   // Set this solver's parameters
            }

            /** @todo Set timeout */
            // planner.local_solver->_params.solver_timeout(1. / (config_->clock_frequency_) - (ros::Time::now() - data.control_loop_time_).seconds() - 0.005);
            planner.local_solver->_params.solver_timeout = 1. / CONFIG["control_frequency"].as<double>() - 20. / 1000.;
            // planner.local_solver->_params.
            // SOLVE OPTIMIZATION
            // planner.local_solver->printParameters(1);
            LOG_DEBUG("Planner [" << planner.id << "]: Solving ...");
            planner.result.exit_code = solver->solve();
            // solver_results_[i].exit_code =ellipsoidal_constraints_[solver->solver_id_].Optimize(solver.get()); // IF THIS OPTIMIZATION EXISTS!
            LOG_DEBUG("Planner [" << planner.id << "]: Done! (exitcode = " << planner.result.exit_code << ")");

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
                // planner.result.objective *= (1. - global_guidance_->GetConfig()->selection_weight_consistency_); // Discount percentage wise
            }
        }

        omp_set_dynamic(1);

        // // DECISION MAKING
        best_planner_index_ = FindBestPlanner();
        if (best_planner_index_ == -1)
        {
            LOG_WARN("Failed to find a feasible trajectory in any of the " << planners_.size() << " optimizations.");
            return planners_[0].result.exit_code;
        }

        auto &best_planner = planners_[best_planner_index_];
        auto &best_solver = best_planner.local_solver;
        // LOG_INFO("Best Planner ID: " << best_planner.id);

        // // VISUALIZATION
        best_planner.guidance_constraints->visualize(data);
        best_planner.safety_constraints->visualize(data);
        // // Communicate to the guidance which topology class we follow (none if it was the original planner)
        global_guidance_->OverrideSelectedTrajectory(best_planner.result.guidance_ID, best_planner.is_original_planner);

        _solver->_output = best_solver->_output; // Load the solution into the main lmpcc solver
        _solver->_info = best_solver->_info;
        _solver->_params = best_solver->_params;
        // // Now that we have copied the best solver data (which will get propagated), propagate the solvers here
        // for (auto &planner : planners_)
        // {
        //     if (planner.result.success)
        //         planner.local_solver->LoadSolution(config_->shift_plan_forward_);
        // }

        return best_planner.result.exit_code; // Return its exit code
    }

    void GuidanceConstraints::initializeSolverWithGuidance(LocalPlanner &planner)
    {
        auto &solver = planner.local_solver;

        // // Initialize the solver with the guidance trajectory
        // RosTools::CubicSpline2D<tk::spline> &trajectory_spline = global_guidance_->GetGuidanceTrajectory(solver->_solver_id).spline.GetTrajectory();
        RosTools::CubicSpline2D<tk::spline> &trajectory_spline = global_guidance_->GetGuidanceTrajectory(planner.id).spline.GetTrajectory();

        // Initialize the solver in the selected local optimum
        // I.e., set for each k, x(k), y(k) ...
        // The time indices are wrong here I think
        for (int k = 1; k < solver->N; k++) // note that the 0th velocity is the current velocity
        {
            // int index = k + 1;
            int index = k + 1;
            Eigen::Vector2d cur_position = trajectory_spline.GetPoint((double)(index)*solver->dt); // The plan is one ahead
            // global_guidance_->ProjectToFreeSpace(cur_position, k + 1);
            solver->setEgoPrediction(k, "x", cur_position(0));
            solver->setEgoPrediction(k, "y", cur_position(1));

            Eigen::Vector2d cur_velocity = trajectory_spline.GetVelocity((double)(index)*solver->dt); // The plan is one ahead
            solver->setEgoPrediction(k, "psi", std::atan2(cur_velocity(1), cur_velocity(0)));
            solver->setEgoPrediction(k, "v", cur_velocity.norm());
        }
    }

    // void GuidanceConstraints::SetParameters(LocalPlanner &planner, const RealTimeData &data, int N_iter, int &param_idx)
    // {
    //     // Tell each solver how much time it has

    //     /** @todo fix in ROS2 */
    //     // planner.local_solver->setTimeout(1. / (config_->clock_frequency_) - (rclcpp::Clock().now() - data.control_loop_time_).seconds() - 0.005);
    //     // config_->solver_timeout_ / 1000.); // Limit the solver to 40 ms

    //     // Load parameters

    //     planner.guidance_constraints->SetParameters(planner.local_solver.get(), data, N_iter, param_idx);

    //     planner.safety_constraints->SetParameters(planner.local_solver.get(), data, N_iter, param_idx);
    // }

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
    void GuidanceConstraints::visualize(const RealTimeData &data)
    {
        (void)data;
        LOG_DEBUG("Guidance Constraints: Visualize()");

        // Contouring::Visualize();

        // global_guidance_->Visualize(highlight_selected_guidance_, visualized_guidance_trajectory_nr_);
        global_guidance_->Visualize(false, -1);
        for (size_t i = 0; i < planners_.size(); i++)
        {
            auto &planner = planners_[i];
            if (planner.disabled)
                continue;

            // Visualize the warmstart
            Trajectory initial_trajectory;
            for (int k = 1; k < planner.local_solver->N; k++)
                initial_trajectory.add(planner.local_solver->getEgoPrediction(k, "x"), planner.local_solver->getEgoPrediction(k, "y"));
            visualizeTrajectory(initial_trajectory, _name + "/warmstart_trajectories", false, 0.4, 20, 20);

            // Visualize the optimized trajectory
            if (planner.result.success)
            {
                Trajectory trajectory;
                for (int k = 1; k < _solver->N; k++)
                    trajectory.add(planner.local_solver->getOutput(k, "x"), planner.local_solver->getOutput(k, "y"));
                visualizeTrajectory(trajectory, _name + "/optimized_trajectories", false, 0.4, planner.result.color, global_guidance_->GetConfig()->n_paths_);
            }

            // if (planner.result.success)
            //     VisualizeOptimizedPlan(planner); // Visualize the plan
        }

        VISUALS.getPublisher(_name + "/optimized_trajectories").publish();
        VISUALS.getPublisher(_name + "/warmstart_trajectories").publish();
    }

    //     void GuidanceConstraints::VisualizeOptimizedPlan(LocalPlanner &planner)
    //     {
    //         RosTools::ROSPointMarker &plan_points = plan_markers_->getNewPointMarker("CYLINDER");
    //         RosTools::ROSPointMarker &ellipse = plan_markers_->getNewPointMarker("CYLINDER");
    //         RosTools::ROSLine &line = plan_markers_->getNewLine();

    //         bool is_selected_solver = planner.id == best_planner_index_;

    //         plan_points.setScale(0.2 * config_->visuals_scale_, 0.2 * config_->visuals_scale_, 0.1e-3);
    //         ellipse.setScale(2 * vehicle_->discs_[0].radius, 2 * vehicle_->discs_[0].radius);
    //         line.setScale(0.15 * config_->visuals_scale_, 0.15 * config_->visuals_scale_);

    //         if (is_selected_solver && highlight_selected_guidance_)
    //         {
    //             VisualizeGMPCCPlan(planner, plan_points, ellipse, line);
    //         }
    //         else
    //         {
    //             if (planner.is_original_planner)
    //                 VisualizeLMPCCPlan(planner, plan_points, ellipse, line);
    //             else
    //                 VisualizeGuidedPlan(planner, plan_points, ellipse, line);
    //         }

    //         if (visualize_warmstart_)
    //             VisualizeWarmstartPlan(planner, plan_points, ellipse, line);
    //     }

    //     void GuidanceConstraints::VisualizeWarmstartPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
    //     {
    //         if (visualized_guidance_trajectory_nr_ != -1 && visualized_guidance_trajectory_nr_ != planner.id)
    //             return;

    //         plan_points.setColor(0., 1., 0.);
    //         ellipse.setColor(0., 1., 0.);
    //         line.setColor(0., 1., 0.);

    //         auto &vehicle_regions = planner.local_solver->InitialVehiclePrediction(); // Get the ego-vehicle predictions
    //         Eigen::Vector2d pose, previous_pose;

    //         double z = 0.6e-1;

    //         for (size_t k = 0; k < vehicle_regions.size(); k++)
    //         {
    //             for (auto &disc : vehicle_regions[k].discs_)
    //             {
    //                 plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
    //                 ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

    //                 if (k >= 1)
    //                     line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

    //                 previous_pose = disc.AsVector2d();
    //             }
    //         }
    //     }

    //     void GuidanceConstraints::VisualizeGuidedPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
    //     {
    //         if (visualized_guidance_trajectory_nr_ != -1 && visualized_guidance_trajectory_nr_ != planner.id)
    //             return;

    //         int color_idx = planner.result.color;
    //         double alpha = 0.15;

    //         plan_points.setColorInt(color_idx, global_guidance_->GetConfig()->n_paths_); // global_guidance_->NumberOfGuidanceTrajectories());
    //         ellipse.setColorInt(color_idx, global_guidance_->GetConfig()->n_paths_, alpha);
    //         line.setColorInt(color_idx, global_guidance_->GetConfig()->n_paths_);

    //         auto &vehicle_regions = planner.local_solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
    //         Eigen::Vector2d pose, previous_pose;

    //         double z = 0.6e-1;

    //         for (size_t k = 0; k < vehicle_regions.size(); k++)
    //         {
    //             for (auto &disc : vehicle_regions[k].discs_)
    //             {
    //                 plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
    //                 ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

    //                 if (k >= 1)
    //                     line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

    //                 previous_pose = disc.AsVector2d();
    //             }
    //         }
    //     }

    //     void GuidanceConstraints::VisualizeGMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
    //     {
    //         if (visualized_guidance_trajectory_nr_ != -1 && visualized_guidance_trajectory_nr_ != planner.id)
    //             return;

    //         plan_points.setColorInt(2, 0.8, RosTools::Colormap::BRUNO);
    //         ellipse.setColorInt(2, 0.8, RosTools::Colormap::BRUNO);
    //         line.setColorInt(2, 0.8, RosTools::Colormap::BRUNO);

    //         auto &vehicle_regions = planner.local_solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
    //         Eigen::Vector2d pose, previous_pose;

    //         double z = 0.2;

    //         for (size_t k = 0; k < vehicle_regions.size(); k++)
    //         {
    //             for (auto &disc : vehicle_regions[k].discs_)
    //             {
    //                 plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
    //                 ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

    //                 if (k >= 1)
    //                     line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

    //                 previous_pose = disc.AsVector2d();
    //             }
    //         }
    //     }

    //     void GuidanceConstraints::VisualizeLMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
    //     {

    //         double alpha = 0.6;

    //         plan_points.setColorInt(3, alpha, RosTools::Colormap::BRUNO);
    //         ellipse.setColorInt(3, alpha, RosTools::Colormap::BRUNO);
    //         line.setColorInt(3, alpha, RosTools::Colormap::BRUNO);

    //         auto &vehicle_regions = planner.local_solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
    //         Eigen::Vector2d pose, previous_pose;

    //         double z = 0.5;

    //         for (size_t k = 0; k < vehicle_regions.size(); k++)
    //         {
    //             for (auto &disc : vehicle_regions[k].discs_)
    //             {
    //                 plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
    //                 // ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

    //                 if (k >= 1)
    //                     line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

    //                 previous_pose = disc.AsVector2d();
    //             }
    //         }
    //     }

    //     /** @brief Load obstacles into the Homotopy module */
    void GuidanceConstraints::onDataReceived(RealTimeData &data, std::string &&data_name)
    {
        if (data_name == "reference_path") // New
        {
            LOG_DEBUG("Received Reference Path");

            _spline = std::make_unique<Spline2D>(data.reference_path.x, data.reference_path.y); // Construct a spline from the given points
        }

        //         // See if there is any data for the reference path
        //         Contouring::OnDataReceived(solver_interface, data, std::forward<std::string>(data_name));

        // We wait for both the obstacles and state to arrive before we compute here
        if (data_name == "dynamic obstacles")
        {
            LOG_DEBUG("Guidance Constraints: Received dynamic obstacles");
#pragma omp parallel for num_threads(8)
            for (auto &planner : planners_)
            {
                planner.safety_constraints->onDataReceived(data, std::forward<std::string>(data_name));
            }

            std::vector<GuidancePlanner::Obstacle> obstacles;
            for (auto &obstacle : data.dynamic_obstacles)
            {
                std::vector<Eigen::Vector2d> positions;
                size_t k;
                positions.push_back(obstacle.position); /** @note Strange that we need k = 0 here */

                for (k = 0; k < std::max(obstacle.prediction.steps.size(), (size_t)GuidancePlanner::Config::N); k++)
                    positions.push_back(obstacle.prediction.steps[k].position);

                obstacles.emplace_back(obstacle.index, positions, obstacle.radius + CONFIG["robot_radius"].as<double>());
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
    // } void GuidanceConstraints::VisualizeOptimizedPlan(LocalPlanner &planner)
    //     {
    //         RosTools::ROSPointMarker &plan_points = plan_markers_->getNewPointMarker("CYLINDER");
    //         RosTools::ROSPointMarker &ellipse = plan_markers_->getNewPointMarker("CYLINDER");
    //         RosTools::ROSLine &line = plan_markers_->getNewLine();

    //         bool is_selected_solver = planner.id == best_planner_index_;

    //         plan_points.setScale(0.2 * config_->visuals_scale_, 0.2 * config_->visuals_scale_, 0.1e-3);
    //         ellipse.setScale(2 * vehicle_->discs_[0].radius, 2 * vehicle_->discs_[0].radius);
    //         line.setScale(0.15 * config_->visuals_scale_, 0.15 * config_->visuals_scale_);

    //         if (is_selected_solver && highlight_selected_guidance_)
    //         {
    //             VisualizeGMPCCPlan(planner, plan_points, ellipse, line);
    //         }
    //         else
    //         {
    //             if (planner.is_original_planner)
    //                 VisualizeLMPCCPlan(planner, plan_points, ellipse, line);
    //             else
    //                 VisualizeGuidedPlan(planner, plan_points, ellipse, line);
    //         }

    //         if (visualize_warmstart_)
    //             VisualizeWarmstartPlan(planner, plan_points, ellipse, line);
    //     }

    //     void GuidanceConstraints::VisualizeWarmstartPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
    //     {
    //         if (visualized_guidance_trajectory_nr_ != -1 && visualized_guidance_trajectory_nr_ != planner.id)
    //             return;

    //         plan_points.setColor(0., 1., 0.);
    //         ellipse.setColor(0., 1., 0.);
    //         line.setColor(0., 1., 0.);

    //         auto &vehicle_regions = planner.local_solver->InitialVehiclePrediction(); // Get the ego-vehicle predictions
    //         Eigen::Vector2d pose, previous_pose;

    //         double z = 0.6e-1;

    //         for (size_t k = 0; k < vehicle_regions.size(); k++)
    //         {
    //             for (auto &disc : vehicle_regions[k].discs_)
    //             {
    //                 plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
    //                 ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

    //                 if (k >= 1)
    //                     line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

    //                 previous_pose = disc.AsVector2d();
    //             }
    //         }
    //     }

    //     void GuidanceConstraints::VisualizeGuidedPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
    //     {
    //         if (visualized_guidance_trajectory_nr_ != -1 && visualized_guidance_trajectory_nr_ != planner.id)
    //             return;

    //         int color_idx = planner.result.color;
    //         double alpha = 0.15;

    //         plan_points.setColorInt(color_idx, global_guidance_->GetConfig()->n_paths_); // global_guidance_->NumberOfGuidanceTrajectories());
    //         ellipse.setColorInt(color_idx, global_guidance_->GetConfig()->n_paths_, alpha);
    //         line.setColorInt(color_idx, global_guidance_->GetConfig()->n_paths_);

    //         auto &vehicle_regions = planner.local_solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
    //         Eigen::Vector2d pose, previous_pose;

    //         double z = 0.6e-1;

    //         for (size_t k = 0; k < vehicle_regions.size(); k++)
    //         {
    //             for (auto &disc : vehicle_regions[k].discs_)
    //             {
    //                 plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
    //                 ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

    //                 if (k >= 1)
    //                     line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

    //                 previous_pose = disc.AsVector2d();
    //             }
    //         }
    //     }

    //     void GuidanceConstraints::VisualizeGMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
    //     {
    //         if (visualized_guidance_trajectory_nr_ != -1 && visualized_guidance_trajectory_nr_ != planner.id)
    //             return;

    //         plan_points.setColorInt(2, 0.8, RosTools::Colormap::BRUNO);
    //         ellipse.setColorInt(2, 0.8, RosTools::Colormap::BRUNO);
    //         line.setColorInt(2, 0.8, RosTools::Colormap::BRUNO);

    //         auto &vehicle_regions = planner.local_solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
    //         Eigen::Vector2d pose, previous_pose;

    //         double z = 0.2;

    //         for (size_t k = 0; k < vehicle_regions.size(); k++)
    //         {
    //             for (auto &disc : vehicle_regions[k].discs_)
    //             {
    //                 plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
    //                 ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

    //                 if (k >= 1)
    //                     line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

    //                 previous_pose = disc.AsVector2d();
    //             }
    //         }
    //     }

    //     void GuidanceConstraints::VisualizeLMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line)
    //     {

    //         double alpha = 0.6;

    //         plan_points.setColorInt(3, alpha, RosTools::Colormap::BRUNO);
    //         ellipse.setColorInt(3, alpha, RosTools::Colormap::BRUNO);
    //         line.setColorInt(3, alpha, RosTools::Colormap::BRUNO);

    //         auto &vehicle_regions = planner.local_solver->OptimizedVehiclePredictions(); // Get the ego-vehicle predictions
    //         Eigen::Vector2d pose, previous_pose;

    //         double z = 0.5;

    //         for (size_t k = 0; k < vehicle_regions.size(); k++)
    //         {
    //             for (auto &disc : vehicle_regions[k].discs_)
    //             {
    //                 plan_points.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z));
    //                 // ellipse.addPointMarker(Eigen::Vector3d(disc.x, disc.y, z - 2e-3));

    //                 if (k >= 1)
    //                     line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), z - 1e-3), Eigen::Vector3d(disc.x, disc.y, z - 1e-3));

    //                 previous_pose = disc.AsVector2d();
    //             }
    //         }
    //     }

    //     /** @brief Load obstacles into the Homotopy module */
    //     void GuidanceConstraints::OnDataReceived(SolverInterface *solver_interface, RealTimeData &data, std::string &&data_name)
    //     {
    //         if (data_name == "reference_path") // New
    //         {
    //             LOG_INFO("Received Reference Path");

    //             _spline = std::make_unique<Spline2D>(data.reference_path.x, data.reference_path.y); // Construct a spline from the given points
    //         }

    //         // See if there is any data for the reference path
    //         Contouring::OnDataReceived(solver_interface, data, std::forward<std::string>(data_name));

    //         // We wait for both the obstacles and state to arrive before we compute here
    //         if (data_name == "Dynamic Obstacles")
    //         {
    //             LMPCC_INFO(logger_, "Guidance Constraints: Received dynamic obstacles")
    // #pragma omp parallel for num_threads(8)
    //             for (auto &planner : planners_)
    //             {
    //                 planner.safety_constraints->OnDataReceived(planner.local_solver.get(), data, std::forward<std::string>(data_name));
    //             }

    //             std::vector<GuidancePlanner::Obstacle> obstacles;
    //             for (auto &obstacle : data.dynamic_obstacles_)
    //             {
    //                 std::vector<Eigen::Vector2d> positions;
    //                 size_t k;
    //                 positions.push_back(Eigen::Vector2d(obstacle.pose_.position.x, obstacle.pose_.position.y)); // Insert this at k = 0, so that
    //                                                                                                             // we can plan from k=1 onwards

    //                 // If we are computing for the next step, then ignore the first prediction?

    //                 for (k = 0; k < std::max(obstacle.prediction_.gaussians[0].mean.poses.size(), (size_t)GuidancePlanner::Config::N); k++)
    //                 {
    //                     // std::cout << "[" << k << "]: (" << obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.x << ", " << obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.y << std::endl;
    //                     positions.push_back(Eigen::Vector2d(obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.x,
    //                                                         obstacle.prediction_.gaussians[0].mean.poses[k].pose.position.y));
    //                 }

    //                 obstacles.emplace_back(obstacle.id_, positions, obstacle.discs_[0].radius + vehicle_->discs_[0].radius); /** @todo Include the robot
    //                                                                                                                               position */
    //             }

    //             global_guidance_->LoadObstacles(obstacles, data.halfspaces_[0]);
    //         }
    //         else if (data_name == "State")
    //         {
    //             global_guidance_->SetStart(Eigen::Vector2d(solver_->State().x(), solver_->State().y()), solver_->State().psi(),
    //                                        solver_->State().v());
    //         }
    //     }

    //     void GuidanceConstraints::OnReset(SolverInterface *solver_interface)
    //     {
    //         Contouring::OnReset(solver_interface);

    //         global_guidance_->Reset();
    //     }

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
}