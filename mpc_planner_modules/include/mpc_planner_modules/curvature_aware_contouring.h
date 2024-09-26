#ifndef __CURVATURE_AWARE_CONTOURING_H_
#define __CURVATURE_AWARE_CONTOURING_H_

#include <mpc_planner_modules/contouring.h>

#include <ros_tools/spline.h>

namespace MPCPlanner
{
    class CurvatureAwareContouring : public Contouring
    {
    public:
        CurvatureAwareContouring(std::shared_ptr<Solver> solver);

    public:
        void setParameters(const RealTimeData &data, const ModuleData &module_data, int k) override;

    private:
    };
}
#endif // __CURVATURE_AWARE_CONTOURING_H_