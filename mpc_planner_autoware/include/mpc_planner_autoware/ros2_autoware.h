#ifndef AUTOWARE_PLANNER_H
#define AUTOWARE_PLANNER_H

#include <rclcpp/rclcpp.hpp>

#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_solver/state.h>

#include <mpc_planner_msgs/msg/obstacle_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

// #include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <memory>

namespace MPCPlanner
{
  class Planner;
}

namespace RosTools
{
  class Benchmarker;
}

using namespace MPCPlanner;
class Reconfigure;

// using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_vehicle_msgs::msg::SteeringReport;

class AutowarePlanner : public rclcpp::Node
{
public:
  AutowarePlanner();
  ~AutowarePlanner();

  void initializeSubscribersAndPublishers();
  void initializeParameterCallbacks();
  void setRobotRegion();
  void startEnvironment();

  void Loop(); // Main loop

  // Message callbacks
  void stateCallback(nav_msgs::msg::Odometry::SharedPtr msg);
  void steeringCallback(SteeringReport::SharedPtr msg);
  void pathCallback(PathWithLaneId::SharedPtr msg);
  void obstacleCallback(mpc_planner_msgs::msg::ObstacleArray::SharedPtr msg);
  void autowareStatusCallback(OperationModeState::SharedPtr msg);

  // Parameter callbacks
  void resetSimulationCallback(const rclcpp::Parameter &p);

  void actuate();
  void actuateBackup();

private:
  std::unique_ptr<Planner> _planner;

  RealTimeData _data;
  State _state;

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Time _state_received_time;

  std::unique_ptr<Reconfigure> _reconfigure;

  std::unique_ptr<RosTools::Benchmarker> _benchmarker;

  // Subscribers and publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _state_sub;
  rclcpp::Subscription<PathWithLaneId>::SharedPtr _path_sub;
  rclcpp::Subscription<mpc_planner_msgs::msg::ObstacleArray>::SharedPtr _obstacle_sim_sub;
  rclcpp::Subscription<SteeringReport>::SharedPtr _steering_sub;
  rclcpp::Subscription<OperationModeState>::SharedPtr _autoware_status_sub;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr _trajectory_pub;

  // Pedestrian Simulator
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _ped_reset_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _ped_horizon_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_integrator_step_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_clock_frequency_pub;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _ped_start_client;

  // Parameter updates
  std::shared_ptr<rclcpp::ParameterEventHandler> _param_subscriber;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> _param_cb_handles;

  Eigen::Vector2d mapToVehicleCenter(const Eigen::Vector2d &autoware_pos, const double angle) const;
  Eigen::Vector2d mapFromVehicleCenter(const Eigen::Vector2d &center_pos, const double angle) const;

  bool isPathTheSame(PathWithLaneId::SharedPtr path);

  void visualize();
};

#endif // AutowarePlanner_PLANNER_H
