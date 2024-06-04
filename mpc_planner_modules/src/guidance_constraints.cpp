#include "mpc_planner_modules/guidance_constraints.h"

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/data_visualization.h>

#include <guidance_planner/global_guidance.h>

#include <ros_tools/visuals.h>
#include <ros_tools/profiling.h>
#include <ros_tools/data_saver.h>
#include <ros_tools/math.h>

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

        global_guidance_ = std::make_shared<GuidancePlanner::GlobalGuidance>();
        GuidancePlanner::Config::debug_visuals_ = CONFIG["debug_visuals"].as<bool>();

        global_guidance_->SetPlanningFrequency(CONFIG["control_frequency"].as<double>());

        _use_tmpcpp = CONFIG["t-mpc"]["use_t-mpc++"].as<bool>();
        _enable_constraints = CONFIG["t-mpc"]["enable_constraints"].as<bool>();
        _control_frequency = CONFIG["control_frequency"].as<double>();
        _planning_time = 1. / _control_frequency;

        // Initialize the constraint modules
        int n_solvers = global_guidance_->GetConfig()->n_paths_; // + 1 for the main lmpcc solver?

        ROSTOOLS_ASSERT(n_solvers > 0 || _use_tmpcpp, "Guidance constraints cannot run with 0 paths and T-MPC++ disabled!");

        LOG_VALUE("Solvers", n_solvers);
        for (int i = 0; i < n_solvers; i++)
        {
            planners_.emplace_back(i);
        }

        if (_use_tmpcpp) // ADD IT AS FIRST PLAN
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

        if (module_data.path == nullptr)
        {
            LOG_MARK("Path data not yet available");
            return;
        }

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

        if (_use_tmpcpp && global_guidance_->GetConfig()->n_paths_ == 0) // No global guidance
            return;

        // Set the goals of the global guidance planner
        global_guidance_->SetStart(state.getPos(), state.get("psi"), state.get("v"));

        if (module_data.path_velocity != nullptr)
            global_guidance_->SetReferenceVelocity(module_data.path_velocity->operator()(state.get("spline")));
        else
            global_guidance_->SetReferenceVelocity(CONFIG["weights"]["reference_velocity"].as<double>());

        if (!CONFIG["enable_output"].as<bool>())
        {
            LOG_INFO_THROTTLE(15000, "Not propagating nodes (output is disabled)");
            global_guidance_->DoNotPropagateNodes();
        }

        // Set the goals for the guidance planner
        setGoals(state, module_data);

        LOG_MARK("Running Guidance Search");
        global_guidance_->Update(); /** @note The main update */

        mapGuidanceTrajectoriesToPlanners();

        // LOG_VALUE("Number of Guidance Trajectories", global_guidance_->NumberOfGuidanceTrajectories());
        empty_data_ = data;
        empty_data_.dynamic_obstacles.clear();
    }

    void GuidanceConstraints::setGoals(State &state, const ModuleData &module_data)
    {
        LOG_MARK("Setting guidance planner goals");

        double current_s = state.get("spline");
        double robot_radius = CONFIG["robot_radius"].as<double>();

        if (module_data.path_velocity == nullptr || module_data.path_width_left == nullptr || module_data.path_width_right == nullptr)
        {
            global_guidance_->LoadReferencePath(std::max(0., state.get("spline")), module_data.path,
                                                CONFIG["road"]["width"].as<double>() / 2. - robot_radius - 0.1,
                                                CONFIG["road"]["width"].as<double>() / 2. - robot_radius - 0.1);
            return;
        }

        // Define goals along the reference path, taking into account the velocity along the path
        double final_s = current_s;
        for (int k = 1; k < global_guidance_->GetConfig()->N; k++) // Euler integrate the velocity along the path
            final_s += module_data.path_velocity->operator()(final_s) * _solver->dt;

        int n_long = global_guidance_->GetConfig()->longitudinal_goals_;
        int n_lat = global_guidance_->GetConfig()->vertical_goals_;

        ROSTOOLS_ASSERT((n_lat % 2) == 1, "Number of lateral grid points should be odd!");
        ROSTOOLS_ASSERT(n_long >= 2, "There should be at least two longitudinal goals (start, end)");

        int middle_lat = (n_lat - 1) / 2;
        std::vector<double> s_long = RosTools::linspace(current_s, final_s, n_long);

        ROSTOOLS_ASSERT(s_long[1] - s_long[0] > 0.05, "Goals should have some spacing between them (Config::reference_velocity_ should not be zero)");

        double long_best = s_long.back();

        std::vector<GuidancePlanner::Goal> goals;
        for (int i = 0; i < n_long; i++)
        {
            double s = s_long[i]; // Distance along the path for these goals

            // Compute its cost (distance to the desired goal)
            double long_cost = std::abs(s - long_best);

            // Compute the normal vector to the reference path
            Eigen::Vector2d line_point = module_data.path->getPoint(s);
            Eigen::Vector2d normal = module_data.path->getOrthogonal(s);

            // Place goals orthogonally to the path
            std::vector<double> dist_lat = RosTools::linspace(-module_data.path_width_left->operator()(s) + robot_radius,
                                                              module_data.path_width_right->operator()(s) - robot_radius,
                                                              n_lat);
            // Put the middle goal on the reference path
            dist_lat[middle_lat] = 0.0;

            for (int j = 0; j < n_lat; j++)
            {
                if (i == 0 && j != middle_lat)
                    continue; // Only the first goal should be in the center

                double d = dist_lat[j];

                double lat_cost = std::abs(d);                                     // Higher cost, the further away from the center line
                goals.emplace_back(line_point + normal * d, long_cost + lat_cost); // Add the goal
            }
        }

        global_guidance_->SetGoals(goals);
    }

    void GuidanceConstraints::mapGuidanceTrajectoriesToPlanners()
    {
        // Map each of the found guidance trajectories to an optimization ID
        // Maintaining the same homotopy class so that its initialization is valid

        std::vector<int> remaining_trajectories;
        for (size_t p = 0; p < planners_.size(); p++)
        {
            planners_[p].taken = false;
            planners_[p].existing_guidance = false;
        }
        _map_homotopy_class_to_planner.clear();

        for (int i = 0; i < global_guidance_->NumberOfGuidanceTrajectories(); i++)
        {
            int homotopy_class = global_guidance_->GetGuidanceTrajectory(i).topology_class;
            // LOG_VALUE("Homotopy Class", homotopy_class);

            // Does it match any of the planners?
            bool planner_found = false;
            for (size_t p = 0; p < planners_.size(); p++)
            {
                /** @note More than one guidance trajectory may map to the same planner */
                if (planners_[p].result.guidance_ID == homotopy_class && !planners_[p].taken)
                {
                    _map_homotopy_class_to_planner[i] = p;
                    planners_[p].taken = true;
                    planners_[p].existing_guidance = true;
                    planner_found = true;
                    // LOG_INFO("Planner " << p << " reserved for homotopy class " << homotopy_class);
                    break;
                }
            }

            if (!planner_found)
                remaining_trajectories.push_back(i);
        }

        // Assign the remaining trajectories to the remaining planners
        for (int i : remaining_trajectories)
        {
            for (size_t p = 0; p < planners_.size(); p++)
            {
                if (!planners_[p].taken)
                {
                    _map_homotopy_class_to_planner[i] = p;
                    planners_[p].taken = true;
                    planners_[p].existing_guidance = false;
                }
            }
        }

        // Debug: Log the result
        // for (int i = 0; i < global_guidance_->NumberOfGuidanceTrajectories(); i++)
        // {
        //     LOG_VALUE("Map Guidance Plan" << i,
        //               _map_homotopy_class_to_planner[i]);
        // }
    }

    void GuidanceConstraints::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
    {
        (void)module_data;
        (void)data;
        if (k == 0)
        {
            _solver->_params.solver_timeout = 0.02; // Should not do anything

            LOG_MARK("Guidance Constraints does not need to set parameters");
        }
    }

    int GuidanceConstraints::optimize(State &state, const RealTimeData &data, ModuleData &module_data)
    {
        PROFILE_FUNCTION();
        // Required for parallel call to the solvers when using Forces
        omp_set_nested(1);
        omp_set_max_active_levels(2);
        omp_set_dynamic(0);
        LOG_MARK("Guidance Constraints: optimize");

        if (!_use_tmpcpp && !global_guidance_->Succeeded())
            return 0;

        bool shift_forward = CONFIG["shift_previous_solution_forward"].as<bool>() &&
                             CONFIG["enable_output"].as<bool>();

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
            if (planner.is_original_planner || (!_enable_constraints))
            {
                planner.guidance_constraints->update(state, empty_data_, module_data);
                planner.safety_constraints->update(state, data, module_data); // Updates collision avoidance constraints
            }
            else
            {
                LOG_MARK("Planner [" << planner.id << "]: Loading guidance into the solver and constructing constraints");

                if (planner.existing_guidance)
                    planner.local_solver->initializeWarmstart(state, shift_forward);
                else
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

            // Set timeout (Planning time - used time - time necessary afterwards)
            std::chrono::duration<double> used_time = std::chrono::system_clock::now() - data.planning_start_time;
            planner.local_solver->_params.solver_timeout = _planning_time - used_time.count() - 0.006;

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
                    planner.result.objective *= global_guidance_->GetConfig()->selection_weight_consistency_;
            }
        }

        omp_set_dynamic(1);

        {
            PROFILE_SCOPE("Decision");
            // DECISION MAKING
            best_planner_index_ = FindBestPlanner();
            if (best_planner_index_ == -1)
            {
                LOG_MARK("Failed to find a feasible trajectory in any of the " << std::to_string(planners_.size()) << " optimizations.");
                return planners_[0].result.exit_code;
            }

            auto &best_planner = planners_[best_planner_index_];
            auto &best_solver = best_planner.local_solver;
            // LOG_INFO("Best Planner ID: " << best_planner.id);

            // Communicate to the guidance which topology class we follow (none if it was the original planner)
            global_guidance_->OverrideSelectedTrajectory(best_planner.result.guidance_ID, best_planner.is_original_planner);

            _solver->_output = best_solver->_output; // Load the solution into the main lmpcc solver
            _solver->_info = best_solver->_info;
            _solver->_params = best_solver->_params;

            return best_planner.result.exit_code; // Return its exit code
        }
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
        if (!(_use_tmpcpp && global_guidance_->GetConfig()->n_paths_ == 0)) // If global guidance
            global_guidance_->Visualize(CONFIG["t-mpc"]["highlight_selected"].as<bool>(), -1);
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
            if (CONFIG["debug_visuals"].as<bool>())
            {
                Trajectory initial_trajectory;
                for (int k = 1; k < planner.local_solver->N; k++)
                    initial_trajectory.add(planner.local_solver->getEgoPrediction(k, "x"), planner.local_solver->getEgoPrediction(k, "y"));
                visualizeTrajectory(initial_trajectory, _name + "/warmstart_trajectories", false, 0.2, 20, 20);
            }

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
                    visualizeTrajectory(trajectory, _name + "/optimized_trajectories", false, 1.0, planner.result.color, global_guidance_->GetConfig()->n_paths_, true, false);
                // else if (!planner.existing_guidance) // Visualizes new homotopy classes
                // visualizeTrajectory(trajectory, _name + "/optimized_trajectories", false, 0.2, 11, 12, true, false);
            }
        }

        {
            VISUALS.getPublisher(_name + "/optimized_trajectories").publish();
            if (CONFIG["debug_visuals"].as<bool>())
                VISUALS.getPublisher(_name + "/warmstart_trajectories").publish();
        }
    }

    bool GuidanceConstraints::isDataReady(const RealTimeData &data, std::string &missing_data)
    {
        (void)data;

        bool ready = true;

        ready = ready && planners_[0].guidance_constraints->isDataReady(data, missing_data);
        ready = ready && planners_[0].safety_constraints->isDataReady(data, missing_data);

        if (!ready)
            return false;

        return true;
    }

    //     /** @brief Load obstacles into the Homotopy module */
    void GuidanceConstraints::onDataReceived(RealTimeData &data, std::string &&data_name)
    {

        if (data_name == "goal") // New
        {
            LOG_MARK("Goal input is not yet implemented for T-MPC");

            // Eigen::Vector2d start = global_guidance_->GetStart();
            // std::vector<double> x = {start(0), data.goal(0)};
            // std::vector<double> y = {start(1), data.goal(1)};

            // module_data = std::make_unique<RosTools::Spline2D>(x, y); // Construct a spline from the given point

            /** @todo */
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

                for (size_t k = 0; k < obstacle.prediction.modes[0].size(); k++) // std::max(obstacle.prediction.modes[0].size(), (size_t)GuidancePlanner::Config::N); k++)
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
        // _spline.reset(nullptr);
        global_guidance_->Reset();

        for (auto &planner : planners_)
            planner.local_solver->reset();
    }

    void GuidanceConstraints::saveData(RosTools::DataSaver &data_saver)
    {
        data_saver.AddData("runtime_guidance", global_guidance_->GetLastRuntime());
        for (size_t i = 0; i < planners_.size(); i++) // auto &solver : solvers_)
        {
            auto &planner = planners_[i];
            double objective = planner.result.success ? planner.result.objective : -1.;
            data_saver.AddData("objective_" + std::to_string(i), objective);

            if (planner.is_original_planner)
            {
                data_saver.AddData("lmpcc_objective", objective);
                data_saver.AddData("original_planner_id", planner.id); // To identify which one is the original planner
            }
            else
            {

                // for (int k = 1; k < _solver->N; k++)
                // {
                //     data_saver.AddData(
                //         "solver" + std::to_string(i) + "_plan" + std::to_string(k),
                //         Eigen::Vector2d(_solver->getOutput(k, "x"), _solver->getOutput(k, "y")));
                // }
            }
            // data_saver.AddData("active_constraints_" + std::to_string(planner.id), planner.guidance_constraints->NumActiveConstraints(planner.local_solver.get()));
        }

        data_saver.AddData("best_planner_idx", best_planner_index_);
        double best_objective = best_planner_index_ != -1 ? planners_[best_planner_index_].local_solver->_info.pobj : -1.;

        data_saver.AddData("gmpcc_objective", best_objective);

        global_guidance_->saveData(data_saver); // Save data from the guidance planner
    }
} // namespace MPCPlanner