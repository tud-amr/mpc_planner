#ifndef __ROS1_JACKAL_PLANNER_H__
#define __ROS1_JACKAL_PLANNER_H__

#include <mpc_planner_jackalsimulator/jackalsimulator_reconfigure.h>
#include <mpc_planner_solver/solver_interface.h>
#include <mpc_planner_types/realtime_data.h>

#include <mpc_planner_msgs/ObstacleArray.h> /** @Todo: Replace! */

#include <ros_tools/profiling.h>

#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <derived_object_msgs/ObjectArray.h>
#include <sensor_msgs/Joy.h>

#include <std_srvs/Empty.h>
#include <robot_localization/SetPose.h>

#include <tf2_ros/transform_broadcaster.h>

#include <memory>

using namespace MPCPlanner;

namespace MPCPlanner
{
    class Planner;
}
class JackalPlanner
{
public:
    JackalPlanner(ros::NodeHandle &nh);
    ~JackalPlanner();

    void initializeSubscribersAndPublishers(ros::NodeHandle &nh);

    void startEnvironment();
    bool objectiveReached();

    void loop(const ros::TimerEvent &event);

    void stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg); /** @note: Connects to the JackalSimulator */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg);

    void collisionCallback(const std_msgs::Float64::ConstPtr &msg);

    void reset(bool success = true);

    void publishPose();
    void publishCamera();

private:
    std::unique_ptr<Planner> _planner;

    std::unique_ptr<JackalsimulatorReconfigure> _reconfigure;

    RealTimeData _data;
    State _state;

    ros::Timer _timer;

    bool _enable_output{false};

    RosTools::Timer _timeout_timer;

    std::unique_ptr<RosTools::Benchmarker> _benchmarker;

    // Subscribers and publishers
    ros::Subscriber _state_sub, _state_pose_sub;
    ros::Subscriber _goal_sub;
    ros::Subscriber _path_sub;
    ros::Subscriber _obstacle_sub, _obstacle_sim_sub;
    ros::Subscriber _collisions_sub;

    ros::Publisher _ped_horizon_pub, _ped_integrator_step_pub, _ped_clock_frequency_pub;
    ros::ServiceClient _ped_start_client;

    ros::Publisher _cmd_pub;
    ros::Publisher _pose_pub;

    tf2_ros::TransformBroadcaster _camera_pub;
    ros::Time _prev_stamp;

    std_srvs::Empty _reset_msg;
    robot_localization::SetPose _reset_pose_msg;
    ros::Publisher _reset_simulation_pub;
    ros::ServiceClient _reset_simulation_client;
    ros::ServiceClient _reset_ekf_client;

    bool isPathTheSame(const nav_msgs::Path::ConstPtr &path);

    void visualize();
};

#endif // __ROS1_JACKAL_PLANNER_H__
