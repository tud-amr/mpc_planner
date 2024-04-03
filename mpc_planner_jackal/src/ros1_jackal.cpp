#include <mpc_planner_jackal/ros1_jackal.h>

#include <mpc_planner/data_preparation.h>

#include <ros_tools/visuals.h>
#include <mpc_planner_util/parameters.h>
#include <ros_tools/logging.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/profiling.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>

#include <std_msgs/Empty.h>

using namespace MPCPlanner;

JackalPlanner::JackalPlanner(ros::NodeHandle &nh)
{

    LOG_INFO("Started Jackal Planner");

    // Initialize the configuration
    Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

    _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())};

    // Initialize the planner
    _planner = std::make_unique<Planner>();

    // Initialize the ROS interface
    initializeSubscribersAndPublishers(nh);

    _reconfigure = std::make_unique<JackalReconfigure>();

    _benchmarker = std::make_unique<RosTools::Benchmarker>("loop");

    RosTools::Instrumentor::Get().BeginSession("mpc_planner_jackal");

    // Start the control loop
    _timer = nh.createTimer(
        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()),
        &JackalPlanner::loop,
        this);

    LOG_DIVIDER();
}

JackalPlanner::~JackalPlanner()
{
    LOG_INFO("Stopped Jackal Planner");
    RosTools::Instrumentor::Get().EndSession();
}

void JackalPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
{
    LOG_INFO("initializeSubscribersAndPublishers");

    _state_sub = nh.subscribe<nav_msgs::Odometry>(
        "/input/state", 5,
        boost::bind(&JackalPlanner::stateCallback, this, _1));

    _state_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/input/state_pose", 5,
        boost::bind(&JackalPlanner::statePoseCallback, this, _1));

    _goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/input/goal", 1,
        boost::bind(&JackalPlanner::goalCallback, this, _1));

    _path_sub = nh.subscribe<nav_msgs::Path>(
        "/input/reference_path", 1,
        boost::bind(&JackalPlanner::pathCallback, this, _1));

    _obstacle_sub = nh.subscribe<derived_object_msgs::ObjectArray>(
        "/input/obstacles", 1,
        boost::bind(&JackalPlanner::obstacleCallback, this, _1));

    _bluetooth_sub = nh.subscribe<sensor_msgs::Joy>(
        "/input/bluetooth", 1,
        boost::bind(&JackalPlanner::bluetoothCallback, this, _1)); // Deadman switch

    _cmd_pub = nh.advertise<geometry_msgs::Twist>(
        "/output/command", 1);

    // Roadmap reverse
    _reverse_roadmap_pub = nh.advertise<std_msgs::Empty>("/roadmap/reverse", 1);
}

bool JackalPlanner::objectiveReached()
{
    bool reset_condition_forward_x = (_forward_x_experiment) && (_state.get("x") > 3.5 || _state.get("y") > 2.6);
    bool reset_condition_backward_x = (!_forward_x_experiment) && (_state.get("x") < -3.5 || _state.get("y") < -2.6);
    bool reset_condition = reset_condition_forward_x || reset_condition_backward_x;
    if (reset_condition)
    {
        _forward_x_experiment = !_forward_x_experiment;
    }
    return reset_condition;
}

void JackalPlanner::loop(const ros::TimerEvent &event)
{
    (void)event;
    LOG_MARK("============= Loop =============");

    _benchmarker->start();

    if (objectiveReached())
        reset();

    // Print the state
    if (CONFIG["debug_output"].as<bool>())
        _state.print();

    if (_rotate_to_goal)
    {
        rotateToGoal();
        return;
    }

    auto output = _planner->solveMPC(_state, _data);

    LOG_VALUE_DEBUG("Success", output.success);

    geometry_msgs::Twist cmd;
    if (_enable_output && output.success)
    {
        LOG_MARK("Publishing command");
        // Publish the command
        cmd.linear.x = _planner->getSolution(1, "v");  // = x1
        cmd.angular.z = _planner->getSolution(0, "w"); // = u0
        LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
        LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
    }
    else if (!_enable_output)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }
    else
    {
        double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
        double velocity_after_braking;
        double velocity;
        double dt = 1. / CONFIG["control_frequency"].as<double>();

        velocity = _state.get("v");
        velocity_after_braking = velocity - deceleration * dt; // Brake with the given deceleration
        cmd.linear.x = std::max(velocity_after_braking, 0.);   // Don't drive backwards when braking
        cmd.angular.z = 0.0;
    }
    _cmd_pub.publish(cmd);
    _benchmarker->stop();

    _planner->visualize(_state, _data);
    visualize();

    LOG_DEBUG("============= End Loop =============");
}

void JackalPlanner::rotateToGoal()
{
    LOG_INFO_THROTTLE(500, "Rotating to the goal");
    if (!_data.goal_received)
    {
        LOG_INFO("Waiting for the goal");
        return;
    }

    // LOG_VALUE("goal x", _data.goal(0));
    // LOG_VALUE("goal y", _data.goal(1));

    double goal_angle = std::atan2(_data.goal(1) - _state.get("y"), _data.goal(0) - _state.get("x"));
    double angle_diff = goal_angle - _state.get("psi"); // RosTools::angleDifference(_state.get("psi"), goal_angle);

    // LOG_VALUE("goal angle", goal_angle);

    if (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;

    // LOG_VALUE("diff", angle_diff);
    geometry_msgs::Twist cmd;
    if (std::abs(angle_diff) > 0.1)
    {
        cmd.linear.x = 0.0;
        if (_enable_output)
            cmd.angular.z = 1.5 * RosTools::sgn(angle_diff);
        else
            cmd.angular.z = 0.;

        _cmd_pub.publish(cmd);
    }
    else
    {
        _rotate_to_goal = false;
    }
}

void JackalPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);
    _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
    _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));
}

void JackalPlanner::statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _state.set("x", msg->pose.position.x);
    _state.set("y", msg->pose.position.y);
    _state.set("psi", msg->pose.orientation.z);
    _state.set("v", msg->pose.position.z);
}

void JackalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    LOG_WARN("Goal callback");
    _data.goal(0) = msg->pose.position.x;
    _data.goal(1) = msg->pose.position.y;
    _data.goal_received = true;

    _planner->onDataReceived(_data, "goal");
}

bool JackalPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg)
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

void JackalPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
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

void JackalPlanner::obstacleCallback(const derived_object_msgs::ObjectArray::ConstPtr &msg)
{
    _data.dynamic_obstacles.clear();

    int additions = 0;
    std::vector<double> angles;
    std::vector<Eigen::Vector2d> positions;
    std::vector<double> radii;
    std::vector<Eigen::Vector2d> twists;
    std::vector<ObstacleType> types;

    for (auto &object : msg->objects)
    {
        double object_angle = RosTools::quaternionToAngle(object.pose.orientation) +
                              std::atan2(object.twist.linear.y, object.twist.linear.x) +
                              M_PI_2;
        double velocity = std::sqrt(object.twist.linear.x * object.twist.linear.x +
                                    object.twist.linear.y * object.twist.linear.y);
        if (velocity < 0.01) // Stop the visualization from rotating if standing still
            object_angle = RosTools::quaternionToAngle(object.pose.orientation);

        int prev_size = positions.size();
        parseObstacle(object, object_angle, positions, radii);
        int new_obstacles = positions.size() - prev_size;

        for (int i = 0; i < new_obstacles; i++)
            angles.push_back(object_angle);

        geometry_msgs::Twist global_twist = object.twist;
        Eigen::Matrix2d rot_matrix = RosTools::rotationMatrixFromHeading(-RosTools::quaternionToAngle(object.pose.orientation));
        Eigen::Vector2d twist_out = rot_matrix * Eigen::Vector2d(global_twist.linear.x, global_twist.linear.y);

        for (int i = 0; i < new_obstacles; i++)
            twists.push_back(twist_out);

        // Assume obstacles consisting of multiple parts are static
        if (new_obstacles > 1)
        {
            for (int i = 0; i < new_obstacles; i++)
                types.push_back(ObstacleType::STATIC);
        }
        else
            types.push_back(ObstacleType::DYNAMIC);
    }

    for (int i = 0; i < positions.size(); i++)
    {
        _data.dynamic_obstacles.emplace_back(
            i,
            positions[i],
            angles[i],
            radii[i],
            types[i]);

        auto &dynamic_obstacle = _data.dynamic_obstacles.back();

        dynamic_obstacle.prediction = getConstantVelocityPrediction(
            dynamic_obstacle.position,
            twists[i],
            CONFIG["integrator_step"].as<double>(),
            CONFIG["N"].as<int>());
    }

    // Eigen::Vector2d velocity = Eigen::Vector2d(object.twist.linear.x, object.twist.linear.y);

    ensureObstacleSize(_data.dynamic_obstacles, _state);

    _planner->onDataReceived(_data, "dynamic obstacles");
}

void JackalPlanner::parseObstacle(const derived_object_msgs::Object &object, double object_angle,
                                  std::vector<Eigen::Vector2d> &positions_out, std::vector<double> &radii_out)
{
    // Depending on the shape of the obstacle, interpret it differently
    if (object.shape.type == object.shape.CYLINDER)
    {
        positions_out.emplace_back(object.pose.position.x, object.pose.position.y);
        radii_out.push_back(object.shape.dimensions[1]);
    }
    else if (object.shape.type == object.shape.BOX)
    {
        if (object.shape.dimensions[0] == object.shape.dimensions[1])
        {
            positions_out.emplace_back(object.pose.position.x, object.pose.position.y);
            radii_out.push_back(std::sqrt(2) * object.shape.dimensions[0]);
        }
        else
        {
            Eigen::Vector2d pos(object.pose.position.x, object.pose.position.y);

            double small_dim = std::min(object.shape.dimensions[0], object.shape.dimensions[1]);
            double large_dim = std::max(object.shape.dimensions[0], object.shape.dimensions[1]);
            double margin = 0.05;

            positions_out.emplace_back(pos + Eigen::Vector2d(std::cos(object_angle - M_PI_2),
                                                             std::sin(object_angle - M_PI_2)) *
                                                 large_dim / 2);
            radii_out.emplace_back(std::sqrt(2) * small_dim + margin);

            positions_out.emplace_back(pos - Eigen::Vector2d(std::cos(object_angle - M_PI_2),
                                                             std::sin(object_angle - M_PI_2)) *
                                                 large_dim / 2);
            radii_out.emplace_back(std::sqrt(2) * small_dim + margin);
        }
    }
}

void JackalPlanner::bluetoothCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    if (msg->axes[2] < -0.9 && !_enable_output)
        LOG_INFO("Planning enabled (deadman switch pressed)");
    else if (msg->axes[2] > -0.9 && _enable_output)
        LOG_INFO("Deadmanswitch enabled (deadman switch released)");

    _enable_output = msg->axes[2] < -0.9;
}

void JackalPlanner::visualize()
{
    auto &publisher = VISUALS.getPublisher("angle");
    auto &line = publisher.getNewLine();

    line.addLine(Eigen::Vector2d(_state.get("x"), _state.get("y")),
                 Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")), _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
    publisher.publish();

    auto &goal_publisher = VISUALS.getPublisher("goal");
    auto &cube = goal_publisher.getNewPointMarker("CUBE");

    cube.setScale(0.05, 0.05, 0.2);
    cube.setColorInt(4, 5);

    cube.addPointMarker(Eigen::Vector3d(_data.goal(0), _data.goal(1), 0.0));
    goal_publisher.publish();
}

void JackalPlanner::reset()
{
    LOG_INFO("Resetting");

    std_msgs::Empty empty_msg;
    _reverse_roadmap_pub.publish(empty_msg);

    _planner->reset(_state, _data);
    _rotate_to_goal = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    ros::NodeHandle nh;
    auto jackal_planner = std::make_shared<JackalPlanner>(nh);
    VISUALS.init(&nh);

    ros::spin();

    return 0;
}