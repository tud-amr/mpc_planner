#ifndef VISUALS_H
#define VISUALS_H

#define VISUALS Visuals::get()

// Still requires ROS support for now, but can be modified to use other visualisation tools

#define MPC_PLANNER_ROS 1

#if MPC_PLANNER_ROS == 1
#include <ros_tools/ros_visuals.h>
#include <unordered_map>
class Visuals
{
public:
    static Visuals &get()
    {
        static Visuals instance;
        return instance;
    }

    void init(ros::NodeHandle *nh)
    {
        _name = ros::this_node::getName();
        _nh = nh;
    }

    RosTools::ROSMarkerPublisher &getPublisher(std::string topic_name)
    {
        if (_publishers.find(topic_name) == _publishers.end())
        {
            std::string node_topic_name = _name + std::string("/") + topic_name;
            _publishers.insert(std::make_pair(topic_name, RosTools::ROSMarkerPublisher(*_nh, node_topic_name.c_str(), frame_id, 10)));
        }
        return _publishers.at(topic_name); // Retrieve the publisher from the map
    }

private:
    std::unordered_map<std::string, RosTools::ROSMarkerPublisher> _publishers;

    std::string _name{""};
    ros::NodeHandle *_nh; // ROS1

    std::string frame_id{"map"};

    Visuals() {}                                  // Private constructor to prevent instantiation
    ~Visuals() {}                                 // Private destructor to prevent deletion
    Visuals(const Visuals &) = delete;            // Delete copy constructor
    Visuals &operator=(const Visuals &) = delete; // Delete assignment operator
};

#elif MPC_PLANNER_ROS == 2
#include <ros_tools/ros_visuals.h>
#include <rclcpp/rclcpp.hpp>

class Visuals
{
public:
    static Visuals &get()
    {
        static Visuals instance;
        return instance;
    }

    void init(rclcpp::Node *node)
    {
        _node = node;
    }

    RosTools::ROSMarkerPublisher &getPublisher(std::string topic_name)
    {
        if (_publishers.find(topic_name) == _publishers.end())
        {
            std::string node_topic_name = _node->get_name() + std::string("/") + topic_name;
            _publishers.insert(std::make_pair(topic_name, RosTools::ROSMarkerPublisher(_node, node_topic_name.c_str(), frame_id, 10)));
        }
        return _publishers.at(topic_name); // Retrieve the publisher from the map
    }

private:
    std::unordered_map<std::string, RosTools::ROSMarkerPublisher> _publishers;

    rclcpp::Node *_node{nullptr}; // Necessary for ROS2

    std::string frame_id{"map"};

    Visuals() {}                                  // Private constructor to prevent instantiation
    ~Visuals() {}                                 // Private destructor to prevent deletion
    Visuals(const Visuals &) = delete;            // Delete copy constructor
    Visuals &operator=(const Visuals &) = delete; // Delete assignment operator
};
#endif

// Tools for visualizing custom data types
namespace MPCPlanner
{
    struct Trajectory;
    struct DynamicObstacle;
    RosTools::ROSMarkerPublisher &visualizeTrajectory(const Trajectory &trajectory, const std::string &topic_name,
                                                      bool publish = false, double alpha = 0.4);

    RosTools::ROSMarkerPublisher &visualizeObstacles(const std::vector<DynamicObstacle> &obstacles, const std::string &topic_name,
                                                     bool publish = false, double alpha = 0.6);

    RosTools::ROSMarkerPublisher &visualizeObstaclePredictions(const std::vector<DynamicObstacle> &obstacles, const std::string &topic_name,
                                                               bool publish = false, double alpha = 0.3);

    RosTools::ROSMarkerPublisher &visualizeLinearConstraint(double a1, double a2, double b, int k, int N, const std::string &topic_name,
                                                            bool publish = false, double alpha = 1.0, double thickness = 0.05);
}

#endif // VISUALS_H
