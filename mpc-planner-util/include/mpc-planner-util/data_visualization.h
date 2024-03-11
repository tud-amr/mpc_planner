
#ifndef DATA_VISUALIZATION_H
#define DATA_VISUALIZATION_H

#include <ros_tools/visuals.h>

// Tools for visualizing custom data types
namespace MPCPlanner
{
    struct Trajectory;
    struct DynamicObstacle;
    struct Disc;
    RosTools::ROSMarkerPublisher &visualizeTrajectory(const Trajectory &trajectory, const std::string &topic_name,
                                                      bool publish = false, double alpha = 0.4, int color_index = 0, int color_max = 10,
                                                      bool publish_trace = true, bool publish_regions = true);

    RosTools::ROSMarkerPublisher &visualizeObstacles(const std::vector<DynamicObstacle> &obstacles, const std::string &topic_name,
                                                     bool publish = false, double alpha = 0.6);

    RosTools::ROSMarkerPublisher &visualizeObstaclePredictions(const std::vector<DynamicObstacle> &obstacles, const std::string &topic_name,
                                                               bool publish = false, double alpha = 0.3);

    RosTools::ROSMarkerPublisher &visualizeLinearConstraint(double a1, double a2, double b, int k, int N, const std::string &topic_name,
                                                            bool publish = false, double alpha = 1.0, double thickness = 0.05);

    RosTools::ROSMarkerPublisher &visualizeRobotArea(const Eigen::Vector2d &position, const double angle,
                                                     const std::vector<Disc> robot_area, const std::string &topic_name,
                                                     bool publish = false, double alpha = 1.0);
}

#endif // DATA_VISUALIZATION_H