#ifndef __ROS1_ROSNAVIGATION_PLANNER_H__
#define __ROS1_ROSNAVIGATION_PLANNER_H__

#include <nav_core/base_local_planner.h>

#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>

#include <mpc_planner_rosnavigation/rosnavigation_reconfigure.h>
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

#define CAMERA_BUFFER 10

using namespace MPCPlanner;

namespace MPCPlanner
{
    class Planner;
}
namespace local_planner
{
    class ROSNavigationPlanner : public nav_core::BaseLocalPlanner
    {

    public:
        // ROSNavigationPlanner(ros::NodeHandle &nh);

        // NAVIGATION STACK
        ROSNavigationPlanner();
        ROSNavigationPlanner(std::string name, tf2_ros::Buffer *tf,
                             costmap_2d::Costmap2DROS *costmap_ros);

        ~ROSNavigationPlanner();

        void initialize(std::string name, tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();

        // OTHER STANDARD FUNCTIONS
        void initializeSubscribersAndPublishers(ros::NodeHandle &nh);

        void startEnvironment();
        // bool objectiveReached();

        void loop(geometry_msgs::Twist &cmd_vel);

        void stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg); /** @note: Connects to the Rosnavigation */
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void pathCallback(const nav_msgs::Path::ConstPtr &msg);
        void obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg);

        void collisionCallback(const std_msgs::Float64::ConstPtr &msg);

        void reset(bool success = true);

        void publishPose();
        void publishCamera();

    private:
        // ROS Navigation stack
        costmap_2d::Costmap2DROS *costmap_ros_;
        tf2_ros::Buffer *tf_;
        bool initialized_;
        std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan

        ros::NodeHandle general_nh_;

        // Other

        std::unique_ptr<Planner> _planner;

        std::unique_ptr<RosnavigationReconfigure> _reconfigure;

        RealTimeData _data;
        State _state;

        bool _enable_output{false};

        RosTools::Timer _timeout_timer;

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

        double _x_buffer[CAMERA_BUFFER];
        double _y_buffer[CAMERA_BUFFER];

        bool isPathTheSame(const nav_msgs::Path::ConstPtr &path);

        void visualize();
    };
} // namespace local_planner
#endif // __ROS1_ROSNAVIGATION_PLANNER_H__
