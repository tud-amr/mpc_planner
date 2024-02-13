#include <mpc-planner-jackal/jackal_planner.h>

#include <ros_tools/helpers.h>

using namespace MPCPlanner;
using namespace rclcpp;

JackalPlanner::JackalPlanner() : Node("jackal_planner")
{
    LOG_INFO("Started Jackal Planner");

    // Load parameters from file
    loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "settings"), _config);

    _planner = std::make_unique<Planner>(_config);

    initializeSubscribersAndPublishers();

    _timer = create_timer(
        this,
        this->get_clock(),
        Duration::from_seconds(1.0 / _config["control_frequency"].as<double>()),
        std::bind(&JackalPlanner::Loop, this));
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

    _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        "~/output/command", 1);
}

void JackalPlanner::Loop()
{
    LOG_INFO("============= Loop =============");

    // Print the state
    LOG_VALUE("State x  ", _state.get("x"));
    LOG_VALUE("State y  ", _state.get("y"));
    LOG_VALUE("State psi", _state.get("psi"));
    LOG_VALUE("State v  ", _state.get("v"));

    _planner->solveMPC(_state, _data);

    // Publish the command
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = _planner->getSolution(1, "v");
    cmd.angular.z = _planner->getSolution(0, "w");
    LOG_VALUE("Commanded v", cmd.linear.x);
    LOG_VALUE("Commanded w", cmd.angular.z);
    _cmd_pub->publish(cmd);

    LOG_INFO("============= End Loop =============");
}

void JackalPlanner::stateCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    // LOG_INFO("State callback");
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);
    _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
    _state.set("v", msg->twist.twist.linear.x);
}

void JackalPlanner::goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    LOG_INFO("Goal callback");
    _data.goal(0) = msg->pose.position.x;
    _data.goal(1) = msg->pose.position.y;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto jackal_planner = std::make_shared<JackalPlanner>();

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