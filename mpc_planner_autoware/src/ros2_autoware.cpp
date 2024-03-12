#include <mpc_planner_autoware/ros2_autoware.h>

#include <mpc-planner/data_preparation.h>

#include <mpc-planner-util/parameters.h>
#include <mpc-planner-util/load_yaml.hpp>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>

using namespace MPCPlanner;
using namespace rclcpp;
using namespace std::chrono_literals;

AutowarePlanner::~AutowarePlanner()
{
  RosTools::Instrumentor::Get().EndSession();
}

AutowarePlanner::AutowarePlanner()
    : Node("autoware_planner")
{
  LOG_INFO("Started Autoware Planner");

  RosTools::Instrumentor::Get().BeginSession("mpc_planner_autoware");

  STATIC_NODE_POINTER.init(this);

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

  _benchmarker = std::make_unique<RosTools::Benchmarker>("loop");

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
  LOG_INFO("initializeSubscribersAndPublishers");

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
      "~/input/obstacles", 1,
      std::bind(&AutowarePlanner::obstacleCallback, this, std::placeholders::_1));

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
  // for (int i = 0; i < 20; i++)
  // {

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

  _benchmarker->start();

  // Print the state
  _state.print();

  auto output = _planner->solveMPC(_state, _data);

  LOG_VALUE_DEBUG("Success", output.success);
  // geometry_msgs::msg::Twist cmd;
  if (output.success)
  {
    actuate();
  }
  //   // Publish the command
  //   cmd.linear.x = _planner->getSolution(1, "v");  // = x1
  //   cmd.angular.z = _planner->getSolution(0, "w"); // = u0
  //   LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
  //   LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
  // }
  // else
  // {
  //   cmd.linear.x = 0.0;
  //   cmd.angular.z = 0.0;
  // }
  // _cmd_pub->publish(cmd);
  _benchmarker->stop();

  _planner->visualize(_state, _data);
  visualize();

  LOG_DEBUG("============= End Loop =============");
}

void AutowarePlanner::stateCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  // LOG_INFO("State callback");
  double angle = RosTools::quaternionToAngle(msg->pose.pose.orientation);
  _state.set("x", msg->pose.pose.position.x + std::cos(angle) * (2.445 - 1.1)); // length - com_to_back
  _state.set("y", msg->pose.pose.position.y + std::sin(angle) * (2.445 - 1.1));
  _state.set("psi", angle);
  _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));

  // LOG_VALUE("x", _state.get("x"));
  // LOG_VALUE("y", _state.get("y"));
  // LOG_VALUE("psi", _state.get("psi"));
  // LOG_VALUE("v", _state.get("v"));

  // _state.print();
}

void AutowarePlanner::steeringCallback(SteeringReport::SharedPtr msg)
{
  _state.set("delta", msg->steering_tire_angle);
}

void AutowarePlanner::obstacleCallback(mpc_planner_msgs::msg::ObstacleArray::SharedPtr msg)
{
  _data.dynamic_obstacles.clear();

  for (auto &obstacle : msg->obstacles)
  {
    // Save the obstacle
    _data.dynamic_obstacles.emplace_back(
        obstacle.id,
        Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
        RosTools::quaternionToAngle(obstacle.pose),
        CONFIG["obstacle_radius"].as<double>());
    auto &dynamic_obstacle = _data.dynamic_obstacles.back();

    if (obstacle.probabilities.size() == 0)
    { // No Predictions!
      continue;
    }

    // Save the prediction
    if (obstacle.probabilities.size() == 1)
    { // One mode
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
      {
        dynamic_obstacle.prediction.type = PredictionType::DETERMINISTIC;
      }
      else
      {
        dynamic_obstacle.prediction.type = PredictionType::GAUSSIAN;
      }
    }
    else
    {
      ROSTOOLS_ASSERT(false, "Multiple modes not yet supported");
    }
  }
  ensureObstacleSize(_data.dynamic_obstacles, _state);
  _planner->onDataReceived(_data, "dynamic obstacles");
}

void AutowarePlanner::pathCallback(PathWithLaneId::SharedPtr msg)
{
  LOG_DEBUG("Path callback");

  if (isPathTheSame(msg))
  {
    return;
  }

  _data.reference_path.clear();

  for (auto &point : msg->points)
  {
    _data.reference_path.x.push_back(point.point.pose.position.x);
    _data.reference_path.y.push_back(point.point.pose.position.y);
    _data.reference_path.psi.push_back(0.0);
  }

  _planner->onDataReceived(_data, "reference_path");
}

bool AutowarePlanner::isPathTheSame(PathWithLaneId::SharedPtr msg)
{
  // Check if the path is the same
  if (_data.reference_path.x.size() != msg->points.size())
  {
    return false;
  }

  // Check up to the first two points
  int num_points = std::min(2, (int)_data.reference_path.x.size());
  for (int i = 0; i < num_points; i++)
  {
    if (!_data.reference_path.pointInPath(i,
                                          msg->points[i].point.pose.position.x,
                                          msg->points[i].point.pose.position.y))
    {
      return false;
    }
  }
  return true;
}

void AutowarePlanner::actuate()
{
  auto trajectory_msg = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>();
  trajectory_msg->points.clear();
  int N = CONFIG["N"].as<int>();
  double dt = CONFIG["integrator_step"].as<double>();

  for (int k = 0; k < N - 1; k++) // Start at 1 (to ignore the initial state)
  {

    // Add this step of the solver to the trajectory
    trajectory_msg->points.emplace_back();
    trajectory_msg->points.back().time_from_start = rclcpp::Duration::from_seconds((double)(k + 1) * dt);

    // Copied from lmpcc
    trajectory_msg->points.back().pose.position.x = _planner->getSolution(k + 1, "x");
    trajectory_msg->points.back().pose.position.y = _planner->getSolution(k + 1, "y");
    trajectory_msg->points.back().pose.orientation = RosTools::angleToQuaternion(_planner->getSolution(k + 1, "psi"));
    trajectory_msg->points.back().longitudinal_velocity_mps = _planner->getSolution(k + 1, "v");

    trajectory_msg->points.back().acceleration_mps2 = _planner->getSolution(k, "a"); // Inputs start from each k, i.e., from 0 - (N-1)
    trajectory_msg->points.back().heading_rate_rps = _planner->getSolution(k, "w");
  }

  trajectory_msg->header.frame_id = "map";
  trajectory_msg->header.stamp = this->get_clock()->now();

  _trajectory_pub->publish(*trajectory_msg);
}

void AutowarePlanner::visualize()
{
  auto &publisher = VISUALS.getPublisher("angle");
  auto &line = publisher.getNewLine();

  line.addLine(
      Eigen::Vector2d(_state.get("x"), _state.get("y")),
      Eigen::Vector2d(
          _state.get("x") + 1.0 * std::cos(_state.get("psi")),
          _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
  publisher.publish();
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