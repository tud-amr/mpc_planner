#include "mpc_planner_modules/curvature_aware_contouring.h"

#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <mpc_planner_util/parameters.h>

namespace MPCPlanner
{

    CurvatureAwareContouring::CurvatureAwareContouring(std::shared_ptr<Solver> solver)
        : Contouring(solver)
    {
    }

    void CurvatureAwareContouring::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
    {
        (void)data;
        (void)module_data;

        // Retrieve weights once
        static double contouring_weight, reference_velocity, velocity_weight;
        static double terminal_angle_weight, terminal_contouring_weight;
        if (k == 0)
        {
            contouring_weight = CONFIG["weights"]["contour"].as<double>();

            terminal_angle_weight = CONFIG["weights"]["terminal_angle"].as<double>();
            terminal_contouring_weight = CONFIG["weights"]["terminal_contouring"].as<double>();

            if (_dynamic_velocity_reference)
            {
                velocity_weight = CONFIG["weights"]["velocity"].as<double>();
                reference_velocity = CONFIG["weights"]["reference_velocity"].as<double>();
            }
        }

        {
            setSolverParameterContour(k, _solver->_params, contouring_weight);

            setSolverParameterTerminalAngle(k, _solver->_params, terminal_angle_weight);
            setSolverParameterTerminalContouring(k, _solver->_params, terminal_contouring_weight);

            if (_dynamic_velocity_reference)
            {
                setSolverParameterVelocity(k, _solver->_params, velocity_weight);
                setSolverParameterReferenceVelocity(k, _solver->_params, reference_velocity);
            }
        }

        setSplineParameters(k);
    }
}