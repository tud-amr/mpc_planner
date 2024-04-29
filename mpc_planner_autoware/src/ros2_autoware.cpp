#include <mpc_planner_autoware/ros2_autoware.h>
#include <mpc_planner_autoware/reconfigure.h>
#include <mpc_planner_autoware/actuation.h>
#include <mpc_planner_autoware/utils.h>

#include <mpc_planner/planner.h>
#include <mpc_planner/data_preparation.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/profiling.h>
#include <ros_tools/math.h>

using namespace MPCPlanner;
using namespace rclcpp;
using namespace std::chrono_literals;

AutowarePlanner::~AutowarePlanner()
{
  RosTools::Instrumentor::Get().EndSession();
  BENCHMARKERS.print();
}

AutowarePlanner::AutowarePlanner()
    : Node("autoware_planner")
{
  STATIC_NODE_POINTER.init(this);
  STATIC_NODE_POINTER.init(this);

  LOG_INFO("Started Autoware Planner");

  RosTools::Instrumentor::Get().BeginSession("mpc_planner_autoware");

  // Initialize the configuration
  Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

  _reconfigure = std::make_unique<Reconfigure>(this);

  // Initialize the planner
  _planner = std::make_unique<Planner>();

  // Initialize the ROS interface
  initializeSubscribersAndPublishers();

  initializeParameterCallbacks();

  setRobotRegion();

  startEnvironment();

  // Start the control loop
  _timer = create_timer(
      this,
      this->get_clock(),
      Duration::from_seconds(1.0 / CONFIG["control_frequency"].as<double>()),
      std::bind(&AutowarePlanner::Loop, this));

  LOG_DIVIDER();
}

void AutowarePlanner::initializeSubscribersAndPublishers()
{
  LOG_INITIALIZE("initializeSubscribersAndPublishers");

  _state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/state", 5,
      std::bind(&AutowarePlanner::stateCallback, this, std::placeholders::_1));

  _steering_sub = this->create_subscription<SteeringReport>(
      "~/input/steering", 5,
      std::bind(&AutowarePlanner::steeringCallback, this, std::placeholders::_1));

  _path_sub = this->create_subscription<PathWithLaneId>(
      "~/input/reference_path", 1,
      std::bind(&AutowarePlanner::pathCallback, this, std::placeholders::_1));

  _obstacle_sim_sub = this->create_subscription<mpc_planner_msgs::msg::ObstacleArray>(
      "~/input/simulated_obstacles", 1,
      std::bind(&AutowarePlanner::simulatedObstacleCallback, this, std::placeholders::_1));

  _obstacle_sub = this->create_subscription<TrackedObjects>(
      "~/input/obstacles", 1,
      std::bind(&AutowarePlanner::obstacleCallback, this, std::placeholders::_1));

  _autoware_status_sub = this->create_subscription<OperationModeState>(
      "~/input/autoware_status", 1,
      std::bind(&AutowarePlanner::autowareStatusCallback, this, std::placeholders::_1));

  _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "~/output/command", 1);
  _trajectory_pub = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/output/trajectory", 1);

  // Pedestrian simulator
  _ped_reset_pub = this->create_publisher<std_msgs::msg::Empty>(
      "/pedestrian_simulator/reset", 1);
  _ped_horizon_pub =
      this->create_publisher<std_msgs::msg::Int32>("/pedestrian_simulator/horizon", 1);
  _ped_integrator_step_pub = this->create_publisher<std_msgs::msg::Float32>(
      "/pedestrian_simulator/integrator_step", 1);
  _ped_clock_frequency_pub = this->create_publisher<std_msgs::msg::Float32>(
      "/pedestrian_simulator/clock_frequency", 1);
  _ped_start_client = this->create_client<std_srvs::srv::Empty>("/pedestrian_simulator/start");
  LOG_INITIALIZED();
}

void AutowarePlanner::initializeParameterCallbacks()
{
  _param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);

  this->declare_parameter<bool>("reset_simulation", false);
  _param_cb_handles.emplace_back(_param_subscriber->add_parameter_callback(
      "reset_simulation",
      std::bind(&AutowarePlanner::resetSimulationCallback, this, std::placeholders::_1)));
}

void AutowarePlanner::setRobotRegion()
{
  _data.robot_area = defineRobotArea(CONFIG["robot"]["length"].as<double>(),
                                     CONFIG["robot"]["width"].as<double>(),
                                     CONFIG["n_discs"].as<int>());
}

void AutowarePlanner::startEnvironment()
{
  LOG_INFO("Starting pedestrian simulator");

  while (!_ped_start_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      LOG_ERROR("Interrupted while waiting for the service. Exiting.");
      return;
    }

    LOG_INFO_THROTTLE(3000, "Waiting for pedestrian simulator to start");
    rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1. * 1e9)));
  }

  std_msgs::msg::Int32 horizon_msg;
  horizon_msg.data = CONFIG["N"].as<int>();
  _ped_horizon_pub->publish(horizon_msg);

  std_msgs::msg::Float32 integrator_step_msg;
  integrator_step_msg.data = CONFIG["integrator_step"].as<double>();
  _ped_integrator_step_pub->publish(integrator_step_msg);

  std_msgs::msg::Float32 clock_frequency_msg;
  clock_frequency_msg.data = CONFIG["control_frequency"].as<double>();
  _ped_clock_frequency_pub->publish(clock_frequency_msg);

  auto empty_srv = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = _ped_start_client->async_send_request(empty_srv);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    LOG_INFO("Environment ready.");
    return;
  }
  LOG_ERROR("This should not print");
}

void AutowarePlanner::Loop()
{
  LOG_DEBUG("============= Loop =============");

  BENCHMARKERS.getBenchmarker("loop").start();

  if (CONFIG["debug_output"].as<bool>())
    _state.print();

  if (_planner->isObjectiveReached(_state, _data))
  {
    LOG_SUCCESS("Planner completed its task.");
    _planner->reset(_state, _data); // Remove previous planner instructions
  }

  auto output = _planner->solveMPC(_state, _data);

  LOG_VALUE_DEBUG("Success", output.success);
  if (output.success)
  {
    _trajectory_pub->publish(outputToAutowareTrajectoryMsg(*_planner));
  }
  else
  {
    _trajectory_pub->publish(backupTrajectoryToAutowareTrajectoryMsg(_state));
  }

  BENCHMARKERS.getBenchmarker("loop").stop();

  _data.past_trajectory.add(_state.getPos());
  _planner->visualize(_state, _data);
  visualize();

  LOG_DEBUG("============= End Loop =============");
}

void AutowarePlanner::stateCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  LOG_MARK("State callback");
  double angle = RosTools::quaternionToAngle(msg->pose.pose.orientation);
  Eigen::Vector2d autoware_pos(msg->pose.pose.position.x, msg->pose.pose.position.y);
  Eigen::Vector2d center_pos = mapToVehicleCenter(autoware_pos, angle);

  _state.set("x", center_pos(0));
  _state.set("y", center_pos(1));
  _state.set("psi", angle);
  _state.set("v", msg->twist.twist.linear.x); //, std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));
  _state_received_time = this->get_clock()->now();
}

void AutowarePlanner::steeringCallback(SteeringReport::SharedPtr msg)
{
  LOG_MARK("Steering callback");
  _state.set("delta", msg->steering_tire_angle /*/ 15.06*/);
}

void AutowarePlanner::obstacleCallback(TrackedObjects::SharedPtr msg)
{
  LOG_MARK("Obstacle callback");

  if (CONFIG["use_simulated_obstacles"].as<bool>())
  {
    LOG_WARN_THROTTLE(10000, "Ignoring real obstacles because use_simulated_obstacles is true");
    return;
  }
  _data.dynamic_obstacles.clear();

  for (auto &obstacle : msg->objects)
  {
    // Save the obstacle
    double angle = RosTools::quaternionToAngle(obstacle.kinematics.pose_with_covariance.pose);
    _data.dynamic_obstacles.emplace_back(
        obstacle.object_id.uuid[0],
        Eigen::Vector2d(obstacle.kinematics.pose_with_covariance.pose.position.x,
                        obstacle.kinematics.pose_with_covariance.pose.position.y),
        angle,
        CONFIG["obstacle_radius"].as<double>(),
        ObstacleType::DYNAMIC);

    auto &dynamic_obstacle = _data.dynamic_obstacles.back();

    // Rotate the velocity from the body frame to the global frame
    Eigen::Vector2d vel = Eigen::Vector2d(obstacle.kinematics.twist_with_covariance.twist.linear.x, 0.);
    vel = RosTools::rotationMatrixFromHeading(-angle) * vel;

    dynamic_obstacle.prediction = getConstantVelocityPrediction(
        dynamic_obstacle.position,
        vel,
        CONFIG["integrator_step"].as<double>(),
        CONFIG["N"].as<int>());
  }
  removeDistantObstacles(_data.dynamic_obstacles, _state);
  ensureObstacleSize(_data.dynamic_obstacles, _state);

  if (CONFIG["probabilistic"]["propagate_uncertainty"].as<bool>())
    propagatePredictionUncertainty(_data.dynamic_obstacles);

  _planner->onDataReceived(_data, "dynamic obstacles");
}

void AutowarePlanner::simulatedObstacleCallback(mpc_planner_msgs::msg::ObstacleArray::SharedPtr msg)
{
  if (!CONFIG["use_simulated_obstacles"].as<bool>()) // Real objects take precedence
    return;

  LOG_MARK("Obstacle callback");

  _data.dynamic_obstacles.clear();

  for (auto &obstacle : msg->obstacles)
  {
    // Save the obstacle
    _data.dynamic_obstacles.emplace_back(
        obstacle.id,
        Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
        RosTools::quaternionToAngle(obstacle.pose),
        CONFIG["obstacle_radius"].as<double>(),
        ObstacleType::DYNAMIC);

    auto &dynamic_obstacle = _data.dynamic_obstacles.back();

    if (obstacle.probabilities.size() == 0) // No Predictions!
      continue;

    // Save the prediction
    if (obstacle.probabilities.size() == 1) // One mode
    {
      dynamic_obstacle.prediction = Prediction(PredictionType::GAUSSIAN);

      const auto &mode = obstacle.gaussians[0];
      for (size_t k = 0; k < mode.mean.poses.size(); k++)
      {
        dynamic_obstacle.prediction.modes[0].emplace_back(
            Eigen::Vector2d(mode.mean.poses[k].pose.position.x, mode.mean.poses[k].pose.position.y),
            RosTools::quaternionToAngle(mode.mean.poses[k].pose.orientation),
            mode.major_semiaxis[k],
            mode.minor_semiaxis[k]);
      }

      if (mode.major_semiaxis.back() == 0. || !CONFIG["probabilistic"]["enable"].as<bool>())
        dynamic_obstacle.prediction.type = PredictionType::DETERMINISTIC;
      else
        dynamic_obstacle.prediction.type = PredictionType::GAUSSIAN;
    }
    else
    {
      ROSTOOLS_ASSERT(false, "Multiple modes not yet supported");
    }
  }

  removeDistantObstacles(_data.dynamic_obstacles, _state);
  ensureObstacleSize(_data.dynamic_obstacles, _state);

  if (CONFIG["probabilistic"]["propagate_uncertainty"].as<bool>())
    propagatePredictionUncertainty(_data.dynamic_obstacles);

  _planner->onDataReceived(_data, "dynamic obstacles");
}

void AutowarePlanner::pathCallback(PathWithLaneId::SharedPtr msg)
{
  LOG_MARK("Path callback");

  if (isPathTheSame(msg, _data))
    return;

  _data.reference_path.clear();
  _data.left_bound.clear();
  _data.right_bound.clear();

  for (auto &point : msg->points)
  {
    _data.reference_path.x.push_back(point.point.pose.position.x);
    _data.reference_path.y.push_back(point.point.pose.position.y);
    _data.reference_path.psi.push_back(0.0);
  }

  if (CONFIG["contouring"]["use_map_bounds"].as<bool>())
  {
    if (!msg->left_bound.empty())
    {
      for (auto &point : msg->left_bound) // Left
      {
        _data.left_bound.x.push_back(point.x);
        _data.left_bound.y.push_back(point.y);
      }
      for (auto &point : msg->right_bound) // Right
      {
        _data.right_bound.x.push_back(point.x);
        _data.right_bound.y.push_back(point.y);
      }
    }
  }

  _planner->onDataReceived(_data, "reference_path");
}

// bool AutowarePlanner::isPathTheSame(PathWithLaneId::SharedPtr msg)
// {
//   // Check if the path is the same
//   if (_data.reference_path.x.size() != msg->points.size())
//   {
//     return false;
//   }

//   // Check up to the first two points
//   int num_points = std::min(2, (int)_data.reference_path.x.size());
//   for (int i = 0; i < num_points; i++)
//   {
//     if (!_data.reference_path.pointInPath(i,
//                                           msg->points[i].point.pose.position.x,
//                                           msg->points[i].point.pose.position.y))
//     {
//       return false;
//     }
//   }
//   return true;
// }

void AutowarePlanner::autowareStatusCallback(OperationModeState::SharedPtr msg)
{
  // Ensures that the planner knows whether its outputs will be executed or not
  CONFIG["enable_output"] = msg->mode == msg->AUTONOMOUS;
}

// void AutowarePlanner::actuate()
// {
//   auto trajectory_msg = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>();
//   trajectory_msg->points.clear();
//   int N = CONFIG["N"].as<int>();
//   double dt = CONFIG["integrator_step"].as<double>();

//   for (int k = 0; k < N - 1; k++)
//   {
//     trajectory_msg->points.emplace_back();
//     trajectory_msg->points.back().time_from_start = rclcpp::Duration::from_seconds((double)(k + 1) * dt);

//     double angle = _planner->getSolution(k + 1, "psi");
//     Eigen::Vector2d center_pos(_planner->getSolution(k + 1, "x"), _planner->getSolution(k + 1, "y"));

//     // We map here the center state (used in our model) to the autoware position in the center of the rear axel
//     Eigen::Vector2d autoware_pos = mapFromVehicleCenter(center_pos, angle);
//     trajectory_msg->points.back().pose.position.x = autoware_pos(0);
//     trajectory_msg->points.back().pose.position.y = autoware_pos(1);
//     trajectory_msg->points.back().pose.orientation = RosTools::angleToQuaternion(angle);
//     trajectory_msg->points.back().longitudinal_vel ocity_mps = _planner->getSolution(k + 1, "v");

//     // Inputs start from each k, i.e., from 0 - (N-1)
//     trajectory_msg->points.back().acceleration_mps2 = _planner->getSolution(k, "a");
//     trajectory_msg->points.back().heading_rate_rps = _planner->getSolution(k, "w");
//   }

//   trajectory_msg->header.frame_id = "map";
//   trajectory_msg->header.stamp = this->now(); // Trajectory is defined from the last state for which we computed the trajectory

//   _trajectory_pub->publish(*trajectory_msg);
// }

// void AutowarePlanner::actuateBackup()
// {
//   auto trajectory_msg = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>();
//   trajectory_msg->points.clear();
//   int N = CONFIG["N"].as<int>();
//   double dt = CONFIG["integrator_step"].as<double>();

//   double x = _state.get("x");
//   double y = _state.get("y");
//   double psi = _state.get("psi");
//   double v = _state.get("v");

//   double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();

//   /** @note: Returning 1e-2 instead of zero prevents Autoware from choking */
//   auto positive_velocity_lambda = [](double v)
//   { return std::max(v, 1e-2); };

//   // x = x, y = y, psi = psi
//   v = positive_velocity_lambda(v - deceleration * dt); // Update for the initial state

//   for (int k = 0; k < N - 1; k++)
//   {
//     trajectory_msg->points.emplace_back();
//     trajectory_msg->points.back().time_from_start = rclcpp::Duration::from_seconds((double)(k + 1) * dt);

//     x += v * std::cos(psi) * dt;
//     y += v * std::sin(psi) * dt;
//     Eigen::Vector2d center_pos(x, y);

//     // We map here the center state (used in our model) to the autoware position in the center of the rear axel
//     Eigen::Vector2d autoware_pos = mapFromVehicleCenter(center_pos, psi);

//     trajectory_msg->points.back().pose.position.x = autoware_pos(0);
//     trajectory_msg->points.back().pose.position.y = autoware_pos(1);
//     trajectory_msg->points.back().pose.orientation = RosTools::angleToQuaternion(psi);
//     trajectory_msg->points.back().longitudinal_velocity_mps = v;

//     v = positive_velocity_lambda(v - deceleration * dt); // Brake with deceleration
//   }

//   trajectory_msg->header.frame_id = "map";
//   trajectory_msg->header.stamp = this->now(); // Trajectory is defined from the last state for which we computed the trajectory

//   _trajectory_pub->publish(*trajectory_msg);
// }

void AutowarePlanner::visualize()
{
  auto &traj_publisher = VISUALS.getPublisher("past_trajectory");
  auto &line = traj_publisher.getNewLine();
  line.setColorInt(6, 10);
  line.setScale(0.1, 0.1);

  Eigen::Vector2d prev;
  for (size_t i = 0; i < _data.past_trajectory.positions.size(); i++)
  {
    if (i > 0)
      line.addLine(prev, _data.past_trajectory.positions[i]);

    prev = _data.past_trajectory.positions[i];
  }
  traj_publisher.publish();
}

void AutowarePlanner::resetSimulationCallback(const rclcpp::Parameter &p)
{
  if (!p.as_bool())
    return;

  LOG_INFO("Resetting simulation (due to rqt_reconfigure input)");

  std_msgs::msg::Empty empty;
  _ped_reset_pub->publish(empty);

  // Set the parameter back to false (we already reset the simulation)
  this->set_parameters({rclcpp::Parameter("reset_simulation", false)});
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto autoware_planner = std::make_shared<AutowarePlanner>();
  VISUALS.init(autoware_planner.get());

  rclcpp::spin(autoware_planner);

  return 0;
}
