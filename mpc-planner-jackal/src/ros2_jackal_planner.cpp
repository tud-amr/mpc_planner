#include <mpc-planner-jackal/jackal_planner.h>

#include <mpc-planner-util/visuals.h>
#include <mpc-planner-util/parameters.h>
#include <mpc-planner-util/logging.h>
#include <mpc-planner-util/load_yaml.hpp>

#include <ros_tools/helpers.h>

using namespace MPCPlanner;
using namespace rclcpp;

JackalPlanner::JackalPlanner() : Node("jackal_planner")
{
    LOG_INFO("Started Jackal Planner");

    // Initialize the configuration
    Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

    // Initialize the planner
    _planner = std::make_unique<Planner>();

    // Initialize the ROS interface
    initializeSubscribersAndPublishers();

    _benchmarker = std::make_unique<RosTools::Benchmarker>("loop");

    // Start the control loop
    _timer = create_timer(
        this,
        this->get_clock(),
        Duration::from_seconds(1.0 / CONFIG["control_frequency"].as<double>()),
        std::bind(&JackalPlanner::Loop, this));

    LOG_DIVIDER();
}

void JackalPlanner::initializeSubscribersAndPublishers()
{
    LOG_INFO("initializeSubscribersAndPublishers");

    _state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/input/state", 5,
        std::bind(&JackalPlanner::stateCallback, this, std::placeholders::_1));

    _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/input/goal", 1,
        std::bind(&JackalPlanner::goalCallback, this, std::placeholders::_1));

    _path_sub = this->create_subscription<nav_msgs::msg::Path>(
        "~/input/reference_path", 1,
        std::bind(&JackalPlanner::pathCallback, this, std::placeholders::_1));

    _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        "~/output/command", 1);
}

void JackalPlanner::Loop()
{
    LOG_DEBUG("============= Loop =============");

    _benchmarker->start();

    // Print the state
    _state.print();

    auto output = _planner->solveMPC(_state, _data);

    LOG_VALUE_DEBUG("Success", output.success);

    geometry_msgs::msg::Twist cmd;
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
    _cmd_pub->publish(cmd);
    _benchmarker->stop();

    _planner->visualize(_state, _data);
    visualize();

    LOG_DEBUG("============= End Loop =============");
}

void JackalPlanner::stateCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    // LOG_INFO("State callback");
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);
    _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
    _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));
}

void JackalPlanner::goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    LOG_DEBUG("Goal callback");
    _data.goal(0) = msg->pose.position.x;
    _data.goal(1) = msg->pose.position.y;
    _data.goal_received = true;
}

bool JackalPlanner::isPathTheSame(nav_msgs::msg::Path::SharedPtr msg)
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

void JackalPlanner::pathCallback(nav_msgs::msg::Path::SharedPtr msg)
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

void JackalPlanner::visualize()
{
    auto &publisher = VISUALS.getPublisher("angle");
    auto &line = publisher.getNewLine();

    line.addLine(Eigen::Vector2d(_state.get("x"), _state.get("y")),
                 Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")), _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
    publisher.publish();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto jackal_planner = std::make_shared<JackalPlanner>();
    VISUALS.init(jackal_planner.get());

    rclcpp::spin(jackal_planner);

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