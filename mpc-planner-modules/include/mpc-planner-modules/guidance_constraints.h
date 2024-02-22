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

#include <mpc-planner-modules/definitions.h>

#include <mpc-planner-modules/linearized_constraints.h>

#include <mpc-planner-types/controller_module.h>
#include <mpc-planner-solver/solver_interface.h>

#include <mpc-planner-util/spline.h>

#include <guidance_planner/global_guidance.h>

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
        void update(State &state, const RealTimeData &data) override;
        void setParameters(const RealTimeData &data, int k) override;

        // bool isDataReady(const RealTimeData &data, std::string &missing_data) override;

        void visualize(const RealTimeData &data) override;

        /**
         * @brief Override to define a custom optimization loop. Note that there can only be ONE customized optimization.
         *
         * @return int exit_code of the solver, return any exit_code other than "EXIT_CODE_NOT_OPTIMIZED_YET" to define this
         * as a custom optimization
         */
        int optimize(State &state, const RealTimeData &data) override; // Default: no custom optimization

        /** @brief Load obstacles into the Homotopy module */
        void onDataReceived(RealTimeData &data, std::string &&data_name) override;

        void reset() override;
        // void ExportData(RosTools::DataSaver &data_saver) override;
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

            LocalPlanner(int _id, bool _is_original_planner = false);
        };

        // void SetGoalCosts();

        // void SetParameters(LocalPlanner &planner, const RealTimeData &data, int N_iter, int &param_idx);

        void initializeSolverWithGuidance(LocalPlanner &planner);

        int FindBestPlanner();

        // void VisualizeOptimizedPlan(LocalPlanner &planner);

        // void VisualizeWarmstartPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line);
        // void VisualizeGuidedPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line);
        // void VisualizeGMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line);
        // void VisualizeLMPCCPlan(LocalPlanner &planner, RosTools::ROSPointMarker &plan_points, RosTools::ROSPointMarker &ellipse, RosTools::ROSLine &line);

    private: // Member variables
        std::vector<LocalPlanner> planners_;

        // std::unique_ptr<RosTools::ROSMarkerPublisher> plan_markers_;

        std::unique_ptr<GuidancePlanner::GlobalGuidance> global_guidance_;

        // To set the goals
        std::unique_ptr<Spline2D> _spline{nullptr};
        std::unique_ptr<RosTools::CubicSpline2D<tk::spline>> _guidance_spline{nullptr}; /**@todo Use Spline2D instead of CubicSpline2D*/

        // Configuration parameters
        // bool add_original_planner_, enable_guidance_constraints_, enable_guidance_warmstart_;
        // bool highlight_selected_guidance_, visualize_warmstart_;
        // int visualized_guidance_trajectory_nr_;

        RealTimeData empty_data_;

        int best_planner_index_ = -1;
    };
} // namespace MPCPlanner
#endif // __GUIDANCE_CONSTRAINTS_H__