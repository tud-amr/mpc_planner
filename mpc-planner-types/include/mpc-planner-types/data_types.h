#ifndef MPC_DATA_TYPES_H
#define MPC_DATA_TYPES_H

#include <Eigen/Dense>

#include <vector>

/** Basic high-level data types for motion planning */

namespace MPCPlanner
{

    struct StaticObstacle
    {
    };

    enum class PredictionType{
        DETERMINISTIC = 0,
        GAUSSIAN,
        NONGAUSSIAN,
        NONE
    };

    struct PredictionStep{

        // Mean
        Eigen::Vector2d position;
        double angle;

        // Covariance
        double major_radius;
        double minor_radius;

        PredictionStep(const Eigen::Vector2d &position, double angle, double major_radius, double minor_radius)
            : position(position), angle(angle), major_radius(major_radius), minor_radius(minor_radius)
        {
        }
    };

    struct Prediction{

        PredictionType type;

        std::vector<PredictionStep> steps;

       Prediction()
            : type(PredictionType::NONE)
        {
        }

        Prediction(PredictionType type)
            : type(type)
        {
        }

 
    };

    struct DynamicObstacle
    {
        int index;

        Eigen::Vector2d position;
        double angle;

        double radius;

        Prediction prediction;

        DynamicObstacle(int _index, const Eigen::Vector2d &_position, double _angle, double _radius)
            : index(_index), position(_position), angle(_angle), radius(_radius)
        {
        }
        
    };

    inline Prediction getConstantVelocityPrediction(const Eigen::Vector2d &position, const Eigen::Vector2d &velocity, double dt, int steps)
    {
        Prediction prediction(PredictionType::DETERMINISTIC);

        for (int i = 0; i < steps; i++)
            prediction.steps.push_back(PredictionStep(position + velocity * dt * i, 0., 0., 0.));

        return prediction;
    }



    struct ReferencePath
    {

        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> psi;

        ReferencePath(int length = 10)
        {
            x.reserve(length);
            y.reserve(length);
            psi.reserve(length);
        };

        void clear()
        {
            x.clear();
            y.clear();
            psi.clear();
        }

        bool pointInPath(int point_num, double other_x, double other_y) const
        {
            return (x[point_num] == other_x && y[point_num] == other_y);
        }

        friend std::ostream &operator<<(std::ostream &out, const ReferencePath &path)
        {
            out << "Path:\n";
            for (size_t i = 0; i < path.x.size(); i++)
            {
                out << "(" << path.x[i] << ", " << path.y[i] << ", " << path.psi[i] << ")" << std::endl;
            }
            return out;
        }
    };

    struct Trajectory
    {
        double dt;
        std::vector<Eigen::Vector2d> positions;

        Trajectory(double dt = 0., int length = 10) : dt(dt)
        {
            positions.reserve(length);
        };

        void add(const Eigen::Vector2d &p)
        {
            positions.push_back(p);
        }

        void add(const double x, const double y)
        {
            positions.push_back(Eigen::Vector2d(x, y));
        }
    };
}

#endif