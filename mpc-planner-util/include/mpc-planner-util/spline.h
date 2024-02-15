#ifndef MPC_PLANNER_UTIL_SPLINE_H
#define MPC_PLANNER_UTIL_SPLINE_H

#include <third_party/tkspline.h>

#include <Eigen/Dense>

namespace MPCPlanner
{
    class Spline2D
    {

    public:
        Spline2D(const std::vector<double> &x, const std::vector<double> &y);

        Eigen::Vector2d getPoint(double s) const;

        /** @brief Check the entire spline for the closest point */
        void findClosestPoint(const Eigen::Vector2d &point, int &segment_out, double &s_out) const;

        void getParameters(int segment_index,
                           double &ax, double &bx, double &cx, double &dx,
                           double &ay, double &by, double &cy, double &dy) const;

        int numSegments() const { return _x_spline.m_x_.size(); }
        double getStartOfSegment(int index) const { return _s_vector[index]; };
        double length() const { return _s_vector.back();}

    private:
        tk::spline _x_spline, _y_spline;

        std::vector<double> _s_vector; // Holds distances at which each spline begins

        void computeDistanceVector(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &out);

        double findClosestSRecursively(const Eigen::Vector2d &point, double low, double high, int num_recursions) const;
    };
};

#endif // MPC_PLANNER_UTIL_SPLINE_H
