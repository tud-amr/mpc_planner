#ifndef JACKAL_PLANNER_H
#define JACKAL_PLANNER_H

#include <mpc-planner/planner.h>

#include <mpc-planner-solver/state.h>
#include <mpc_planner_types/realtime_data.h>

#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#include <mpc_planner_msgs/msg/obstacle_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>

#include <memory>

using namespace MPCPlanner;

class JackalsimulatorReconfigure;

class JackalPlanner : public rclcpp::Node
{
public:
    JackalPlanner();

    void initializeSubscribersAndPublishers();
    void startEnvironment();

    void Loop();

    void stateCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pathCallback(nav_msgs::msg::Path::SharedPtr msg);
    void obstacleCallback(mpc_planner_msgs::msg::ObstacleArray::SharedPtr msg);

private:
    std::unique_ptr<Planner> _planner;

    RealTimeData _data;
    State _state;

    rclcpp::TimerBase::SharedPtr _timer;

    std::unique_ptr<JackalsimulatorReconfigure> _reconfigure;

    std::unique_ptr<RosTools::Benchmarker> _benchmarker;

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;
    rclcpp::Subscription<mpc_planner_msgs::msg::ObstacleArray>::SharedPtr _obstacle_sim_sub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _ped_horizon_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_integrator_step_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_clock_frequency_pub;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _ped_start_client;

    bool isPathTheSame(nav_msgs::msg::Path::SharedPtr path);

    void visualize();
};

#endif // JACKAL_PLANNER_H
