#ifndef PROJECTION_H
#define PROJECTION_H

#include <Eigen/Dense>

namespace MPCPlanner
{
    class DouglasRachford
    {
    public:
        void douglasRachfordProjection(Eigen::Vector2d &p, const Eigen::Vector2d &delta,
                                       const Eigen::Vector2d &anchor, const double r,
                                       const Eigen::Vector2d &start_pose)
        {
            p = (p + reflect(reflect(p, anchor, r, p), delta, r, start_pose)) / 2.0;
        }

    private:
        Eigen::Vector2d project(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const double r, const Eigen::Vector2d &start_pose)
        {
            if (std::sqrt((p - delta).transpose() * (p - delta)) < r)
                return delta - (delta - start_pose) / (std::sqrt((start_pose - delta).transpose() * (start_pose - delta))) * r;
            else
                return p;
        }

        Eigen::Vector2d reflect(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const double r, const Eigen::Vector2d &start_pose)
        {
            return 2.0 * project(p, delta, r, start_pose) - p;
        }
    };
}

#endif // PROJECTION_H