#ifndef __ROS1_JACKAL_PLANNER_H__
#define __ROS1_JACKAL_PLANNER_H__

#include <ros/ros.h>

#include <mpc-planner/planner.h>

#include <mpc-planner-solver/solver_interface.h>

#include <mpc-planner-types/realtime_data.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <mpc_planner_msgs/obstacle_array.h> /** @Todo: Replace! */

#include <ros_tools/helpers.h>

#include <memory>

using namespace MPCPlanner;

class JackalPlanner
{
public:
    JackalPlanner(ros::NodeHandle &nh);

    void initializeSubscribersAndPublishers(ros::NodeHandle &nh);

    void Loop(const ros::TimerEvent &event);

    void stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg); /** @note: Connects to the JackalSimulator */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void obstacleCallback(const mpc_planner_msgs::obstacle_array::ConstPtr &msg);

private:
    std::unique_ptr<Planner> _planner;

    RealTimeData _data;
    State _state;

    ros::Timer _timer;

    std::unique_ptr<RosTools::Benchmarker> _benchmarker;

    // Subscribers and publishers
    ros::Subscriber _state_sub;
    ros::Subscriber _goal_sub;
    ros::Subscriber _path_sub;
    ros::Subscriber _obstacle_sub;

    ros::Publisher _cmd_pub;

    bool isPathTheSame(const nav_msgs::Path::ConstPtr &path);

    void visualize();
};

#endif // __ROS1_JACKAL_PLANNER_H__
