#include <mpc-planner-util/visuals.h>

#include <mpc-planner-types/data_types.h>
#include <mpc-planner-util/parameters.h>

namespace MPCPlanner
{

    RosTools::ROSMarkerPublisher &visualizeTrajectory(const Trajectory &trajectory, const std::string &topic_name,
                                                      bool publish, double alpha, int color_index)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);

        auto &cylinder = publisher.getNewPointMarker("CYLINDER");
        cylinder.setScale(2. * CONFIG["robot_radius"].as<double>(), 2. * CONFIG["robot_radius"].as<double>(), 0.01);
        cylinder.setColorInt(color_index, alpha);

        for (size_t k = 0; k < trajectory.positions.size(); k++)
            cylinder.addPointMarker(trajectory.positions[k], 0.0);

        if (publish)
            publisher.publish();

        return publisher;
    }

    RosTools::ROSMarkerPublisher &visualizeObstacles(const std::vector<DynamicObstacle> &obstacles, const std::string &topic_name,
                                                     bool publish, double alpha)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);
        auto &cylinder = publisher.getNewPointMarker("CYLINDER");

        for (auto &obstacle : obstacles)
        {
            cylinder.setScale(2. * obstacle.radius, 2. * obstacle.radius, 0.01);
            cylinder.setColorInt(obstacle.index, CONFIG["max_obstacles"].as<int>(), alpha);
            cylinder.addPointMarker(obstacle.position, 0.0);
        }

        if (publish)
            publisher.publish();

        return publisher;
    }
    RosTools::ROSMarkerPublisher &visualizeObstaclePredictions(const std::vector<DynamicObstacle> &obstacles, const std::string &topic_name,
                                                               bool publish, double alpha)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);
        auto &cylinder = publisher.getNewPointMarker("CYLINDER");

        for (auto &obstacle : obstacles)
        {
            cylinder.setScale(2. * obstacle.radius, 2. * obstacle.radius, 0.01);
            cylinder.setColorInt(obstacle.index, CONFIG["max_obstacles"].as<int>(), alpha);
            for (size_t k = 0; k < obstacle.prediction.steps.size(); k++)
                cylinder.addPointMarker(obstacle.prediction.steps[k].position, 0.0);
        }

        if (publish)
            publisher.publish();

        return publisher;
    }

    RosTools::ROSMarkerPublisher &visualizeLinearConstraint(double a1, double a2, double b, int k, int N, const std::string &topic_name,
                                                            bool publish, double alpha, double thickness)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);
        auto &line = publisher.getNewLine();
        line.setColorInt(k, N, alpha);
        line.setScale(thickness);

        if (std::abs(a1) < 1e-3 && std::abs(a2) < 1e-3)
            return publisher;

        Eigen::Vector2d p1, p2;
        double line_length = 100.0;

        // If we cant draw in one direction, draw in the other
        if (std::abs(a2) >= 1e-3)
        {
            p1(0) = -line_length;
            p1(1) = (b - a1 * p1(0)) / a2; // (b - a1*x) / a2

            p2(0) = line_length;
            p2(1) = (b - a1 * p2(0)) / a2;
        }
        else
        {
            // Draw the constraint as a line
            p1(1) = -line_length;
            p1(0) = (b - a2 * p1(1)) / a1;

            p2(1) = line_length;
            p2(0) = (b - a2 * p2(1)) / a1;
        }

        line.addLine(p1, p2);

        if (publish)
            publisher.publish();
        return publisher;
    }
} // namespace MPCPlanner