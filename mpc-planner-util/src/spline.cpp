#include "mpc-planner-util/spline.h"

#include <mpc-planner-util/logging.h>

#include <ros_tools/helpers.h>

namespace MPCPlanner
{

    Spline2D::Spline2D(const std::vector<double> &x, const std::vector<double> &y)
    {
        // Compute the distance vector
        computeDistanceVector(x, y, _s_vector);

        // Initialize two splines for x and y
        _x_spline.set_points(_s_vector, x);
        _y_spline.set_points(_s_vector, y);
    }

    Eigen::Vector2d Spline2D::getPoint(double s) const
    {
        return Eigen::Vector2d(_x_spline(s), _y_spline(s));
    }

    // Compute distances between points
    void Spline2D::computeDistanceVector(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &out)
    {
        out.clear();
        out.resize(x.size());
        out[0] = 0.;
        for (size_t i = 1; i < x.size(); i++)
        {
            double dist = std::sqrt(std::pow(x[i] - x[i - 1], 2.) + std::pow(y[i] - y[i - 1], 2.));
            out[i] = out[i - 1] + dist;
        }
    }

    void Spline2D::getParameters(int segment_index,
                                 double &ax, double &bx, double &cx, double &dx,
                                 double &ay, double &by, double &cy, double &dy) const
    {
        _x_spline.getParameters(segment_index, ax, bx, cx, dx);
        _y_spline.getParameters(segment_index, ay, by, cy, dy);
    }

    // Find the distance that we travelled on the spline
    void Spline2D::findClosestPoint(const Eigen::Vector2d &point, int &segment_out, double &s_out) const
    {
        s_out = findClosestSRecursively(point, 0., _s_vector.back(), 0);

        for (size_t i = 0; i < _s_vector.size() - 1; i++)
        {
            if (s_out > _s_vector[i] && s_out < _s_vector[i + 1])
            {
                segment_out = i; // Find the index to match the spline variable computed
                return;
            }
        }

        segment_out = _s_vector.size() - 1;
    }

    double Spline2D::findClosestSRecursively(const Eigen::Vector2d &point, double low, double high, int num_recursions) const
    {
        // Stop after x recursions
        if (num_recursions > 20)
        {
            if (std::abs(high - low) > 1e-3)
                LOG_ERROR("FindClosestSRecursively did not find an accurate s (accuracy = "
                          << std::abs(high - low) << " | tolerance = 1e-3)");
            return (low + high) / 2.;
        }

        // Computes the distance between point "s" on the spline and a vehicle position
        auto dist_to_spline = [&](double s, const Eigen::Vector2d &point)
        {
            return RosTools::dist(getPoint(s), point);
        };

        // Compute a middle s value
        double mid = (low + high) / 2.;

        // Compute the distance to the spline for high/low
        double value_low = dist_to_spline(low, point);
        double value_high = dist_to_spline(high, point);

        // Check the next closest value
        if (value_low < value_high)
            return findClosestSRecursively(point, low, mid, num_recursions + 1);
        else
            return findClosestSRecursively(point, mid, high, num_recursions + 1);
    }

}
