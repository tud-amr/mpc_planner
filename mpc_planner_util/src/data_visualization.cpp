#include <mpc_planner_util/data_visualization.h>

#include <ros_tools/visuals.h>

#include <mpc_planner_types/data_types.h>
#include <mpc_planner_util/parameters.h>

namespace MPCPlanner
{

    RosTools::ROSMarkerPublisher &visualizeTrajectory(const Trajectory &trajectory, const std::string &topic_name,
                                                      bool publish, double alpha, int color_index, int color_max,
                                                      bool publish_trace, bool publish_regions)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);

        auto &cylinder = publisher.getNewPointMarker("CYLINDER");
        cylinder.setScale(2. * CONFIG["robot_radius"].as<double>(), 2. * CONFIG["robot_radius"].as<double>(), 0.01);
        cylinder.setColorInt(color_index, color_max, alpha);

        auto &line = publisher.getNewLine();
        line.setScale(0.15, 0.15);
        line.setColorInt(color_index, color_max);

        Eigen::Vector2d prev;
        for (size_t k = 0; k < trajectory.positions.size(); k++)
        {
            if (publish_regions)
                cylinder.addPointMarker(trajectory.positions[k], 0.0);

            if (k > 0 && publish_trace)
                line.addLine(prev, trajectory.positions[k]);

            prev = trajectory.positions[k];
        }

        if (publish)
            publisher.publish();

        return publisher;
    }

    RosTools::ROSMarkerPublisher &visualizeObstacles(const std::vector<DynamicObstacle> &obstacles, const std::string &topic_name,
                                                     bool publish, double alpha)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);
        auto &ped = publisher.getNewModelMarker();

        for (auto &obstacle : obstacles)
        {
            ped.setColorInt(obstacle.index, alpha, RosTools::Colormap::BRUNO);
            ped.setOrientation(obstacle.angle); // RosTools::quaternionToAngle(plot_pose.orientation));
            ped.addPointMarker(obstacle.position, 0.0);
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
            for (size_t k = 0; k < obstacle.prediction.modes[0].size(); k++)
                cylinder.addPointMarker(obstacle.prediction.modes[0][k].position, 0.0);
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
        double line_length = 1.0e6;

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

    RosTools::ROSMarkerPublisher &visualizeLinearConstraint(const Halfspace &halfspace, int k, int N, const std::string &topic_name,
                                                            bool publish, double alpha, double thickness)
    {
        return visualizeLinearConstraint(halfspace.A(0), halfspace.A(1), halfspace.b, k, N, topic_name, publish, alpha, thickness);
    }

    RosTools::ROSMarkerPublisher &visualizeRobotArea(const Eigen::Vector2d &position, const double angle,
                                                     const std::vector<Disc> robot_area, const std::string &topic_name,
                                                     bool publish, double alpha)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);
        auto &cylinder = publisher.getNewPointMarker("CYLINDER");
        cylinder.setColorInt(0, alpha);

        for (const auto &disc : robot_area)
        {
            cylinder.setScale(disc.radius * 2., disc.radius * 2., 0.01);
            cylinder.addPointMarker(disc.getPosition(position, angle));
        }

        if (publish)
            publisher.publish();

        return publisher;
    }

    RosTools::ROSMarkerPublisher &visualizeRectangularRobotArea(const Eigen::Vector2d &position, const double angle,
                                                                const double width, const double length,
                                                                const std::string &topic_name,
                                                                bool publish, double alpha)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);
        auto &rect = publisher.getNewPointMarker("CUBE");
        rect.setColorInt(1, alpha);
        rect.setScale(width, length, 0.01);
        rect.setOrientation(angle);
        rect.addPointMarker(position);

        if (publish)
            publisher.publish();
        return publisher;
    }

    RosTools::ROSMarkerPublisher &visualizeRobotAreaTrajectory(const Trajectory &trajectory,
                                                               const std::vector<double> angles,
                                                               const std::vector<Disc> robot_area,
                                                               const std::string &topic_name,
                                                               bool publish, double alpha)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);

        auto &cylinder = publisher.getNewPointMarker("CYLINDER");
        cylinder.setScale(2. * CONFIG["robot_radius"].as<double>(), 2. * CONFIG["robot_radius"].as<double>(), 0.01);
        cylinder.setColorInt(0, 10, alpha);

        for (size_t k = 0; k < trajectory.positions.size(); k++)
        {
            for (auto &disc : robot_area)
                cylinder.addPointMarker(disc.getPosition(trajectory.positions[k], angles[k]), 0.0);
        }

        if (publish)
            publisher.publish();

        return publisher;
    }
} // namespace MPCPlanner