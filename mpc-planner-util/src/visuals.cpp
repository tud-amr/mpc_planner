#include <mpc-planner-util/visuals.h>

#include <mpc-planner-types/data_types.h>
#include <mpc-planner-util/parameters.h>

namespace MPCPlanner
{

    RosTools::ROSMarkerPublisher &visualizeTrajectory(const Trajectory &trajectory, const std::string &topic_name, bool publish, double alpha)
    {
        RosTools::ROSMarkerPublisher &publisher = VISUALS.getPublisher(topic_name);

        auto &cylinder = publisher.getNewPointMarker("CYLINDER");
        cylinder.setScale(2. * CONFIG["robot_radius"].as<double>(), 2. * CONFIG["robot_radius"].as<double>(), 0.01);
        cylinder.setColorInt(0, alpha);

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

        for (auto& obstacle : obstacles)
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
       
        for (auto& obstacle : obstacles)
        {
            cylinder.setScale(2. * obstacle.radius, 2. * obstacle.radius, 0.01);
            cylinder.setColorInt(obstacle.index, CONFIG["max_obstacles"].as<int>(), alpha);
            for(size_t k = 0; k < obstacle.prediction.steps.size(); k++)
                cylinder.addPointMarker(obstacle.prediction.steps[k].position, 0.0);
        }

        if (publish)
            publisher.publish();

        return publisher;
    }
}; // namespace MPCPlanner