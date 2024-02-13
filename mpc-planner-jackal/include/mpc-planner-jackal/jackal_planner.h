#ifndef JACKAL_PLANNER_H
#define JACKAL_PLANNER_H

#include <mpc-planner/planner.h>
#include <mpc-planner-solver/solver_interface.h>

#include <mpc-planner-types/realtime_data.h>

#include <mpc-planner-util/load_yaml.hpp>
#include <mpc-planner-util/logging.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

using namespace MPCPlanner;

class JackalPlanner : public rclcpp::Node
{
public:
    JackalPlanner();

    void initializeSubscribersAndPublishers();

    void Loop();

    void stateCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
    YAML::Node _config;

    std::unique_ptr<Planner> _planner;

    RealTimeData _data;
    State _state;

    rclcpp::TimerBase::SharedPtr _timer;

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;
};

#endif // JACKAL_PLANNER_H
