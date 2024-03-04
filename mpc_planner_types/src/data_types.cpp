#include "mpc_planner_types/data_types.h"

/** Basic high-level data types for motion planning */

namespace MPCPlanner
{

    PredictionStep::PredictionStep(const Eigen::Vector2d &position, double angle, double major_radius, double minor_radius)
        : position(position), angle(angle), major_radius(major_radius), minor_radius(minor_radius)
    {
    }

    Prediction::Prediction()
        : type(PredictionType::NONE)
    {
    }

    Prediction::Prediction(PredictionType type)
        : type(type)
    {
    }

    DynamicObstacle::DynamicObstacle(int _index, const Eigen::Vector2d &_position, double _angle, double _radius)
        : index(_index), position(_position), angle(_angle), radius(_radius)
    {
    }

    ReferencePath::ReferencePath(int length)
    {
        x.reserve(length);
        y.reserve(length);
        psi.reserve(length);
    }

    void ReferencePath::clear()
    {
        x.clear();
        y.clear();
        psi.clear();
    }

    bool ReferencePath::pointInPath(int point_num, double other_x, double other_y) const
    {
        return (x[point_num] == other_x && y[point_num] == other_y);
    }

    // friend std::ostream &ReferencePath::operator<<(std::ostream &out, const ReferencePath &path)
    // {
    //     out << "Path:\n";
    //     for (size_t i = 0; i < path.x.size(); i++)
    //     {
    //         out << "(" << path.x[i] << ", " << path.y[i] << ", " << path.psi[i] << ")" << std::endl;
    //     }
    //     return out;
    // }

    Trajectory::Trajectory(double dt, int length) : dt(dt)
    {
        positions.reserve(length);
    }

    void Trajectory::add(const Eigen::Vector2d &p)
    {
        positions.push_back(p);
    }

    void Trajectory::add(const double x, const double y)
    {
        positions.push_back(Eigen::Vector2d(x, y));
    }
}