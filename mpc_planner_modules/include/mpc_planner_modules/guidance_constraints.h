/**
 * @file guidance_constraints.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Controller Module for computing guidance trajectories in the state-space and using them to construct
 * constraints (@see python_forces_code/modules.py)
 * @date 2022-09-23 (documented)
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __GUIDANCE_CONSTRAINTS_H__
#define __GUIDANCE_CONSTRAINTS_H__

#include <mpc_planner_modules/definitions.h>

#include <mpc_planner_modules/linearized_constraints.h>

#include <mpc_planner_modules/controller_module.h>
#include <mpc_planner_solver/solver_interface.h>

#include <unordered_map>

namespace GuidancePlanner
{
    class GlobalGuidance;
}

namespace MPCPlanner
{
    /** @brief Save all the relevant results for a parallel solver in one place */
    struct SolverResult
    {
        int exit_code;
        double objective;
        bool success;

        int guidance_ID;
        int color;

        void Reset()
        {
            success = false;
            objective = 1e10;
            exit_code = -1;

            guidance_ID = -1;
            color = -1;
        }
    };

    /**
     * @brief Homotopy Guidance controller module extends from the reference path ControlModule to implement MPCC over the
     * trajectory but starting from the robot
     */
    class GuidanceConstraints : public ControllerModule
    {
    public:
        GuidanceConstraints(std::shared_ptr<Solver> solver);

    public:
        void update(State &state, const RealTimeData &data, ModuleData &module_data) override;
        void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

        bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

        void visualize(const RealTimeData &data, const ModuleData &module_data) override;

        /**
         * @brief Override to define a custom optimization loop. Note that there can only be ONE customized optimization.
         *
         * @return int exit_code of the solver, return any exit_code other than "EXIT_CODE_NOT_OPTIMIZED_YET" to define this
         * as a custom optimization
         */
        int optimize(State &state, const RealTimeData &data, ModuleData &module_data) override; // Default: no custom optimization

        /** @brief Load obstacles into the Homotopy module */
        void onDataReceived(RealTimeData &data, std::string &&data_name) override;

        void reset() override;
        void saveData(RosTools::DataSaver &data_saver) override;
        // void GetMethodName(std::string &name) override;

    private: // Private functions
        struct LocalPlanner
        {
            int id;
            std::unique_ptr<LinearizedConstraints> guidance_constraints;   // Keep the solver in the topology
            std::unique_ptr<GUIDANCE_CONSTRAINTS_TYPE> safety_constraints; // Avoid collisions

            std::shared_ptr<Solver> local_solver; // Distinct solver for each planner
            SolverResult result;

            bool is_original_planner = false;
            bool disabled = true;

            bool taken = false;
            bool existing_guidance = false;

            LocalPlanner(int _id, bool _is_original_planner = false);
        };

        void setGoals(State &state, const ModuleData &module_data);
        void mapGuidanceTrajectoriesToPlanners();
        void initializeSolverWithGuidance(LocalPlanner &planner);

        int FindBestPlanner();

    private: // Member variables
        std::vector<LocalPlanner> planners_;

        std::shared_ptr<GuidancePlanner::GlobalGuidance> global_guidance_;

        std::unordered_map<int, int> _map_homotopy_class_to_planner;

        // Configuration parameters
        bool _use_tmpcpp{true}, _enable_constraints{true};
        double _control_frequency{20.};
        double _planning_time;

        RealTimeData empty_data_;

        int best_planner_index_ = -1;
    };
} // namespace MPCPlanner
#endif // __GUIDANCE_CONSTRAINTS_H__