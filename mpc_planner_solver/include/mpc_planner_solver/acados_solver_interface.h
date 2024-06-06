#ifndef ACADOS_SOLVER_INTERFACE_H
#define ACADOS_SOLVER_INTERFACE_H

#include <iostream>

#include <mpc_planner_solver/state.h>

#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "acados_solver_Solver.h"

#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/logging.h>

#define NX SOLVER_NX
#define NZ SOLVER_NZ
#define NU SOLVER_NU
#define NP SOLVER_NP
#define NBX SOLVER_NBX
#define NBX0 SOLVER_NBX0
#define NBU SOLVER_NBU
#define NSBX SOLVER_NSBX
#define NSBU SOLVER_NSBU
#define NSH SOLVER_NSH
#define NSG SOLVER_NSG
#define NSPHI SOLVER_NSPHI
#define NSHN SOLVER_NSHN
#define NSGN SOLVER_NSGN
#define NSPHIN SOLVER_NSPHIN
#define NSBXN SOLVER_NSBXN
#define NS SOLVER_NS
#define NSN SOLVER_NSN
#define NG SOLVER_NG
#define NBXN SOLVER_NBXN
#define NGN SOLVER_NGN
#define NY0 SOLVER_NY0
#define NY SOLVER_NY
#define NYN SOLVER_NYN
#define NH SOLVER_NH
#define NPHI SOLVER_NPHI
#define NHN SOLVER_NHN
#define NPHIN SOLVER_NPHIN
#define NR SOLVER_NR

namespace MPCPlanner
{
    struct AcadosParameters
    {
        double xinit[NX]; // Initial state
        // double u0[NU];    // Initial input
        double x0[(NU + NX) * (SOLVER_N + 1)]; // Warmstart: [u0, x0 | u1 x1 | ... | uN xN]

        double all_parameters[NP * SOLVER_N]; // NP parameters for all stages

        double solver_timeout{0.}; // Not functional!

        int *getIdxbx0() { return idxbx0; }
        double *getU0() { return x0; } // Note: should only read the first input from this!

        AcadosParameters()
        {
            for (int i = 0; i < NBX0; i++)
                idxbx0[i] = i;

            // Initialize with zeros
            for (int i = 0; i < NX; i++)
                xinit[i] = 0.;

            for (int i = 0; i < (NU + NX) * (SOLVER_N + 1); i++)
                x0[i] = 0.;

            for (int i = 0; i < NP * SOLVER_N; i++)
                all_parameters[i] = 0.;
        }

        void printParameters()
        {
            std::cout << "Parameters:\n";
            for (int k = 0; k < SOLVER_N; k++)
            {
                for (int i = 0; i < NP; i++)
                {
                    std::cout << all_parameters[k * SOLVER_N + i] << ", ";
                }
                std::cout << "\n";
            }
        }

    private:
        int idxbx0[NBX0]; // Indices of initial conditions
        // double lbx0[NBX0]; // Initial conditions? (lb / ub)
        // double ubx0[NBX0];
    };

    class Solver
    {
    public:
        struct AcadosInfo
        {

            int NTIMINGS;
            double min_time;
            double kkt_norm_inf;
            double elapsed_time;
            int sqp_iter;

            double pobj{0.}; // TODO

            AcadosInfo()
            {
                NTIMINGS = 1;
                min_time = 1e12;
            }

            void print() const
            {
                LOG_INFO("Solver info:");
                LOG_VALUE("SQP iterations", sqp_iter);
                LOG_VALUE("Minimum time for solve", min_time * 1000);
                LOG_VALUE("KKT", kkt_norm_inf);
            }
        };

        struct AcadosOutput
        {
            double xtraj[NX * (SOLVER_N + 1)]; // Initial trajectory x and u (move to output?)
            double utraj[NU * SOLVER_N];

            AcadosOutput()
            {
                for (int i = 0; i < NX * (SOLVER_N + 1); i++)
                    xtraj[i] = 0.;

                for (int i = 0; i < NU * SOLVER_N; i++)
                    utraj[i] = 0.;
            }

            void print()
            {
                printf("\n--- xtraj ---\n");
                d_print_exp_tran_mat(NX, SOLVER_N + 1, xtraj, NX);
                printf("\n--- utraj ---\n");
                d_print_exp_tran_mat(NU, SOLVER_N, utraj, NU);
            }
        };

    private:
        Solver_solver_capsule *_acados_ocp_capsule;
        ocp_nlp_config *_nlp_config;
        ocp_nlp_dims *_nlp_dims;
        ocp_nlp_in *_nlp_in;
        ocp_nlp_out *_nlp_out;
        ocp_nlp_solver *_nlp_solver;
        void *_nlp_opts;

    public:
        int _solver_id;

        AcadosParameters _params;
        AcadosInfo _info;
        AcadosOutput _output;

        int N;
        unsigned int nu;   // Number of control variables
        unsigned int nx;   // Differentiable variables
        unsigned int nvar; // Total variable count
        unsigned int npar; // Parameters per iteration
        double dt;

        YAML::Node _config, _parameter_map, _model_map;

    public:
        Solver(int solver_id = 0);
        ~Solver();

        void reset();

        int solve();

        // PARAMETERS //
        bool hasParameter(std::string &&parameter);
        void setParameter(int k, std::string &&parameter, double value);
        void setParameter(int k, std::string &parameter, double value);
        double getParameter(int k, std::string &&parameter);

        // XINIT //
        void setXinit(std::string &&state_name, double value);
        void setXinit(const State &state);

        // WARMSTART //
        void setEgoPrediction(unsigned int k, std::string &&var_name, double value); // Modify the initial guess
        double getEgoPrediction(unsigned int k, std::string &&var_name);             // Get the initial guess
        void setEgoPredictionPosition(unsigned int k, const Eigen::Vector2d &value); // (same for positions)
        Eigen::Vector2d getEgoPredictionPosition(unsigned int k);

        void loadWarmstart();
        void initializeWithState(const State &initial_state);                               // Load the state for each stage
        void initializeWarmstart(const State &state, bool shift_previous_solution_forward); // Use the previous solution as initial guess
        void initializeWithBraking(const State &initial_state);                             // Load a braking trajectory

        // OUTPUT //
        double getOutput(int k, std::string &&state_name) const;

        // DEBUG //
        std::string explainExitFlag(int exitflag) const;
        void printIfBoundLimited() const;
    };
    /*
    void setForcesParameterAcceleration(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterAngularVelocity(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterContour(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterReferenceVelocity(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterVelocity(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterLag(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterTerminalAngle(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterTerminalContouring(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterSplineXA(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineXB(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineXC(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineXD(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineYA(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineYB(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineYC(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineYD(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineStart(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineVA(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineVB(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineVC(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterSplineVD(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterLinConstraintA1(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterLinConstraintA2(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterLinConstraintB(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterEgoDiscRadius(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterEgoDiscOffset(int k, Solver::AcadosParameters &params, const double value, int index = 0);
    void setForcesParameterEllipsoidObstX(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterEllipsoidObstY(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterEllipsoidObstPsi(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterEllipsoidObstMajor(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterEllipsoidObstMinor(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterEllipsoidObstChi(int k, Solver::AcadosParameters &params, const double value, int index);
    void setForcesParameterEllipsoidObstR(int k, Solver::AcadosParameters &params, const double value, int index);
    */
}

#endif // ACADOS_SOLVER_INTERFACE_H
