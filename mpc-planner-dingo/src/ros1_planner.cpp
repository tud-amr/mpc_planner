#include <mpc-planner-dingo/ros1_planner.h>

#include <mpc-planner/data_preparation.h>

#include <ros_tools/visuals.h>
#include <mpc-planner-util/parameters.h>
#include <ros_tools/logging.h>
#include <mpc-planner-util/load_yaml.hpp>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <ros_tools/helpers.h>

using namespace MPCPlanner;

dingoPlanner::dingoPlanner(ros::NodeHandle &nh)
{

    LOG_INFO("Started dingo Planner");

    // Initialize the configuration
    Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

    // Initialize the planner
    _planner = std::make_unique<Planner>();

    // Initialize the ROS interface
    initializeSubscribersAndPublishers(nh);

    startEnvironment();

    _benchmarker = std::make_unique<RosTools::Benchmarker>("loop");

    // Start the control loop
    _timer = nh.createTimer(
        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()),
        &dingoPlanner::loop,
        this);

    LOG_DIVIDER();
}

void dingoPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
{
    LOG_INFO("initializeSubscribersAndPublishers");

    _state_sub = nh.subscribe<nav_msgs::Odometry>(
        "/input/state", 5,
        boost::bind(&dingoPlanner::stateCallback, this, _1));

    // _state_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    //     "/input/state", 5,
    //     boost::bind(&dingoPlanner::statePoseCallback, this, _1));

    _goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/input/goal", 1,
        boost::bind(&dingoPlanner::goalCallback, this, _1));

    _path_sub = nh.subscribe<nav_msgs::Path>(
        "/input/reference_path", 1,
        boost::bind(&dingoPlanner::pathCallback, this, _1));

    _obstacle_sub = nh.subscribe<mpc_planner_msgs::obstacle_array>(
        "/input/obstacles", 1,
        boost::bind(&dingoPlanner::obstacleCallback, this, _1));

    _cmd_pub = nh.advertise<geometry_msgs::Twist>(
        "/output/command", 1);

    // Environment Reset
    _reset_simulation_pub = nh.advertise<std_msgs::Empty>("/lmpcc/reset_environment", 1);
    _reset_simulation_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    _reset_ekf_client = nh.serviceClient<robot_localization::SetPose>("/set_pose");

    // Pedestrian simulator
    _ped_horizon_pub = nh.advertise<std_msgs::Int32>("/pedestrian_simulator/horizon", 1);
    _ped_integrator_step_pub = nh.advertise<std_msgs::Float32>("/pedestrian_simulator/integrator_step", 1);
    _ped_clock_frequency_pub = nh.advertise<std_msgs::Float32>("/pedestrian_simulator/clock_frequency", 1);
    _ped_start_client = nh.serviceClient<std_srvs::Empty>("/pedestrian_simulator/start");
}

void dingoPlanner::startEnvironment()
{
    LOG_INFO("Starting pedestrian simulator");
    for (int i = 0; i < 20; i++)
    {
        std_msgs::Int32 horizon_msg;
        horizon_msg.data = CONFIG["N"].as<int>();
        _ped_horizon_pub.publish(horizon_msg);

        std_msgs::Float32 integrator_step_msg;
        integrator_step_msg.data = CONFIG["integrator_step"].as<double>();
        _ped_integrator_step_pub.publish(integrator_step_msg);

        std_msgs::Float32 clock_frequency_msg;
        clock_frequency_msg.data = CONFIG["control_frequency"].as<double>();
        _ped_clock_frequency_pub.publish(clock_frequency_msg);

        std_srvs::Empty empty_msg;

        if (_ped_start_client.call(empty_msg))
            break;
        else
        {
            LOG_INFO_THROTTLE(3, "Waiting for pedestrian simulator to start");
            ros::Duration(1.0).sleep();
        }
    }
    LOG_INFO("Environment ready.");
}

void dingoPlanner::loop(const ros::TimerEvent &event)
{
    (void)event;

    LOG_DEBUG("============= Loop =============");

    _benchmarker->start();

    if (_state.get("x") > 32.)
    {
        LOG_INFO("Resetting");
        reset();
    }

    // Print the state
    _state.print();

    auto output = _planner->solveMPC(_state, _data);

    LOG_VALUE_DEBUG("Success", output.success);

    geometry_msgs::Twist cmd;
    if (output.success)
    {
        // Publish the command
        cmd.linear.x = _planner->getSolution(1, "v");  // = x1
        cmd.angular.z = _planner->getSolution(0, "w"); // = u0
        LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
        LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
    }
    else
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }

    _cmd_pub.publish(cmd);
    _benchmarker->stop();

    _planner->visualize(_state, _data);
    visualize();

    LOG_DEBUG("============= End Loop =============");
}

void dingoPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // LOG_INFO("State callback");
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);
    _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
    // _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));
    _state.set("v", msg->twist.twist.linear.x);
}

void dingoPlanner::statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _state.set("x", msg->pose.position.x);
    _state.set("y", msg->pose.position.y);
    _state.set("psi", msg->pose.orientation.z);
    _state.set("v", msg->pose.position.z);
}

void dingoPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    LOG_DEBUG("Goal callback");
    _data.goal(0) = msg->pose.position.x;
    _data.goal(1) = msg->pose.position.y;
    _data.goal_received = true;
}

bool dingoPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg)
{
    // Check if the path is the same
    if (_data.reference_path.x.size() != msg->poses.size())
        return false;

    // Check up to the first two points
    int num_points = std::min(2, (int)_data.reference_path.x.size());
    for (int i = 0; i < num_points; i++)
    {
        if (!_data.reference_path.pointInPath(i, msg->poses[i].pose.position.x, msg->poses[i].pose.position.y))
            return false;
    }
    return true;
}

void dingoPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    LOG_DEBUG("Path callback");

    if (isPathTheSame(msg))
        return;

    _data.reference_path.clear();

    for (auto &pose : msg->poses)
    {
        _data.reference_path.x.push_back(pose.pose.position.x);
        _data.reference_path.y.push_back(pose.pose.position.y);
    }
    _data.reference_path.psi.push_back(0.0);
    _planner->onDataReceived(_data, "reference_path");
}

void dingoPlanner::obstacleCallback(const mpc_planner_msgs::obstacle_array::ConstPtr &msg)
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

        if (obstacle.probabilities.size() == 0) // No Predictions!
            continue;

        // Save the prediction
        if (obstacle.probabilities.size() == 1) // One mode
        {
            const auto &mode = obstacle.gaussians[0];
            for (size_t k = 0; k < mode.mean.poses.size(); k++)
            {
                dynamic_obstacle.prediction.steps.emplace_back(
                    Eigen::Vector2d(mode.mean.poses[k].pose.position.x, mode.mean.poses[k].pose.position.y),
                    RosTools::quaternionToAngle(mode.mean.poses[k].pose.orientation),
                    mode.major_semiaxis[k],
                    mode.minor_semiaxis[k]);
            }

            if (mode.major_semiaxis.back() == 0. || !CONFIG["probabilistic"]["enable"].as<bool>())
                dynamic_obstacle.prediction.type = PredictionType::DETERMINISTIC;
            else
            {
                dynamic_obstacle.prediction.type = PredictionType::GAUSSIAN;
                propagatePredictionUncertainty(dynamic_obstacle.prediction);
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

void dingoPlanner::visualize()
{
    auto &publisher = VISUALS.getPublisher("angle");
    auto &line = publisher.getNewLine();

    line.addLine(Eigen::Vector2d(_state.get("x"), _state.get("y")),
                 Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")), _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
    publisher.publish();
}

void dingoPlanner::reset()
{

    // Reset the environment
    for (int j = 0; j < 1; j++)
    {
        // ActuateBrake(5.0);
        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()).sleep();
    }

    _reset_simulation_client.call(_reset_msg);
    _reset_ekf_client.call(_reset_pose_msg);
    _reset_simulation_pub.publish(std_msgs::Empty());

    _planner->reset(_state, _data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    ros::NodeHandle nh;
    auto dingo_planner = std::make_shared<dingoPlanner>(nh);
    VISUALS.init(&nh);

    ros::spin();

    return 0;
}

/*
// These are tests
Solver solver;

solver.setParameter(0, "acceleration", 3.0);
solver.setParameter(0, "angular_velocity", 4.0);
assert(solver.getParameter(0, "angular_velocity") == 4.0);
assert(solver.getParameter(0, "acceleration") == 3.0);

solver.setXinit("x", 10.0);
solver.setXinit("y", 20.0);
assert(solver._params.xinit[0] == 10.0);
assert(solver._params.xinit[1] == 20.0);

// Test run for the solver
solver.setXinit("x", 0.0);
solver.setXinit("y", 0.0);
solver.setXinit("v", 1.0);

for (int k = 0; k < solver.N; k++)
{
solver.setParameter(k, "acceleration", 1.0);
solver.setParameter(k, "angular_velocity", 3.0);
assert(solver.getParameter(k, "acceleration") == 1.0);
assert(solver.getParameter(k, "angular_velocity") == 3.0);
}

int exitflag = solver.solve();
std::cout << "Exit flag: " << exitflag << std::endl;
assert(exitflag == 1);

// Read the solution
std::cout << solver.getOutput(0, "a") << std::endl;
std::cout << solver.getOutput(0, "w") << std::endl;
std::cout << solver.getOutput(2, "x") << std::endl;
std::cout << solver.getOutput(2, "y") << std::endl;
*/