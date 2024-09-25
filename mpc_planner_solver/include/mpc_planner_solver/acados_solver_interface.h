#ifndef ACADOS_SOLVER_INTERFACE_H
#define ACADOS_SOLVER_INTERFACE_H

#include <iostream>

#include <mpc_planner_solver/state.h>

#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "Solver/acados_solver_Solver.h"

#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/logging.h>

#define NX SOLVER_NX
#define NZ SOLVER_NZ
#define NU SOLVER_NU
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
        // double u0[NU];    // Initial isolver_nput
        double x0[(NU + NX) * (SOLVER_N + 1)]; // Warmstart: [u0, x0 | u1 x1 | ... | uN xN]

        double all_parameters[SOLVER_NP * SOLVER_N]; // SOLVER_NP parameters for all stages

        double solver_timeout{0.}; // Not functional!

        int *getIdxbx0() { return idxbx0; }
        double *getU0() { return x0; } // Note: should only read the first isolver_nput from this!

        AcadosParameters()
        {
            for (int i = 0; i < NBX0; i++)
                idxbx0[i] = i;

            // Initialize with zeros
            for (int i = 0; i < NX; i++)
                xinit[i] = 0.;

            for (int i = 0; i < (NU + NX) * (SOLVER_N + 1); i++)
                x0[i] = 0.;

            for (int i = 0; i < SOLVER_NP * SOLVER_N; i++)
                all_parameters[i] = 0.;
        }

        void printParameters(YAML::Node &parameter_map)
        {
            LOG_HEADER("Parameters");
            for (int k = 0; k < SOLVER_N; k++)
            {
                LOG_HEADER(k);

                for (YAML::const_iterator it = parameter_map.begin(); it != parameter_map.end(); ++it)
                {
                    LOG_VALUE(it->first.as<std::string>(), all_parameters[k * SOLVER_NP + it->second.as<int>()]);
                }
            }
        }

    private:
        int idxbx0[NBX0]; // Indices of initial conditions
    };

    class Solver
    {
    public:
        struct AcadosInfo
        {

            double min_time;
            double kkt_norm_inf;
            double elapsed_time;
            int sqp_iter;
            double nlp_res;
            double solvetime;

            int qp_status;

            double pobj{0.}; // TODO

            AcadosInfo()
            {
                min_time = 1e12;
            }

            void print(Solver_solver_capsule *acados_ocp_capsule) const
            {
                LOG_HEADER("Solver Info");
                LOG_VALUE("SQP iterations", sqp_iter);
                LOG_VALUE("Minimum time for solve [ms]", min_time * 1000);
                LOG_VALUE("KKT", kkt_norm_inf);
                LOG_VALUE("Solve Time [ms]", solvetime * 1000.);
                LOG_VALUE("NLP Residuals", nlp_res);
                Solver_acados_print_stats(acados_ocp_capsule);
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
        Solver_solver_capsule *_acados_ocp_capsule;

    private:
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

        int _num_iterations;

    public:
        Solver(int solver_id = 0);
        ~Solver();

        /** @brief Copy data from another solver. Does not copy solver generic parameters like the horizon N*/
        Solver &operator=(const Solver &rhs);

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
}

#endif // ACADOS_SOLVER_INTERFACE_H
