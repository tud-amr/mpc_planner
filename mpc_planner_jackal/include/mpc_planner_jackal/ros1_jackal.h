#ifndef __ROS1_JACKAL_PLANNER_H__
#define __ROS1_JACKAL_PLANNER_H__

#include <mpc_planner_jackal/jackal_reconfigure.h>

#include <mpc_planner/planner.h>

#include <mpc_planner_solver/solver_interface.h>

#include <mpc_planner_types/realtime_data.h>

#include <mpc_planner_msgs/ObstacleArray.h> /** @Todo: Replace! */

#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <derived_object_msgs/ObjectArray.h>
#include <sensor_msgs/Joy.h>

#include <std_srvs/Empty.h>
#include <robot_localization/SetPose.h>

#include <memory>

using namespace MPCPlanner;

namespace RosTools
{
    class Benchmarker;
}

class JackalPlanner
{
public:
    JackalPlanner(ros::NodeHandle &nh);
    ~JackalPlanner();

    void initializeSubscribersAndPublishers(ros::NodeHandle &nh);

    bool objectiveReached();

    void loop(const ros::TimerEvent &event);

    void stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg); /** @note: Connects to the JackalSimulator */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void obstacleCallback(const derived_object_msgs::ObjectArray::ConstPtr &msg);
    void bluetoothCallback(const sensor_msgs::Joy::ConstPtr &msg);

    void rotateToGoal();
    void reset();

private:
    std::unique_ptr<Planner> _planner;
    std::unique_ptr<JackalReconfigure> _reconfigure;

    RealTimeData _data;
    State _state;

    ros::Timer _timer;

    bool _enable_output{false};
    bool _rotate_to_goal{false};
    bool _forward_x_experiment{true};

    double _measured_velocity{0.};

    std::unique_ptr<RosTools::Benchmarker> _benchmarker;

    // Subscribers and publishers
    ros::Subscriber _state_sub, _state_pose_sub;
    ros::Subscriber _goal_sub;
    ros::Subscriber _path_sub;
    ros::Subscriber _obstacle_sub, _obstacle_sim_sub;
    ros::Subscriber _bluetooth_sub;

    ros::Publisher _reverse_roadmap_pub;

    ros::Publisher _cmd_pub;

    void parseObstacle(const derived_object_msgs::Object &object, double object_angle,
                       std::vector<Eigen::Vector2d> &positions_out, std::vector<double> &radii_out);

    bool isPathTheSame(const nav_msgs::Path::ConstPtr &path);

    void visualize();
};

#endif // __ROS1_JACKAL_PLANNER_H__
