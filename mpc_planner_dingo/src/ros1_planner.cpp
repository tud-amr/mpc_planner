#include <mpc_planner_dingo/ros1_planner.h>

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

DingoPlanner::DingoPlanner(ros::NodeHandle &nh)
{

    LOG_INFO("Started Dingo Planner");

    // Initialize the configuration
    Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

    _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())};

    // Initialize the planner
    _planner = std::make_unique<Planner>();

    // Initialize the ROS interface
    initializeSubscribersAndPublishers(nh);

    _reconfigure = std::make_unique<DingoReconfigure>();

    _benchmarker = std::make_unique<RosTools::Benchmarker>("loop");

    RosTools::Instrumentor::Get().BeginSession("mpc_planner_dingo");

    // Start the control loop
    _timer = nh.createTimer(
        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()),
        &DingoPlanner::loop,
        this);

    LOG_DIVIDER();
}

DingoPlanner::~DingoPlanner()
{
    LOG_INFO("Stopped Dingo Planner");
    RosTools::Instrumentor::Get().EndSession();
}

void DingoPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
{
    LOG_INFO("initializeSubscribersAndPublishers");

    _state_sub = nh.subscribe<nav_msgs::Odometry>(
        "/input/state", 5,
        boost::bind(&DingoPlanner::stateCallback, this, _1));

    _state_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/input/state_pose", 5,
        boost::bind(&DingoPlanner::statePoseCallback, this, _1));

    _goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/input/goal", 1,
        boost::bind(&DingoPlanner::goalCallback, this, _1));

    _path_sub = nh.subscribe<nav_msgs::Path>(
        "/input/reference_path", 1,
        boost::bind(&DingoPlanner::pathCallback, this, _1));

    _obstacle_sub = nh.subscribe<derived_object_msgs::ObjectArray>(
        "/input/obstacles", 1,
        boost::bind(&DingoPlanner::obstacleCallback, this, _1));

    _bluetooth_sub = nh.subscribe<sensor_msgs::Joy>(
        "/input/bluetooth", 1,
        boost::bind(&DingoPlanner::bluetoothCallback, this, _1)); // Deadman switch

    _cmd_pub = nh.advertise<geometry_msgs::Twist>(
        "/output/command", 1);

    // Roadmap reverse
    _reverse_roadmap_pub = nh.advertise<std_msgs::Empty>("/roadmap/reverse", 1);
}

bool DingoPlanner::objectiveReached()
{
    bool reset_condition_forward_x = (_forward_x_experiment) && (_state.get("x") > x_max || _state.get("y") > y_max);
    bool reset_condition_backward_x = (!_forward_x_experiment) && (_state.get("x") < x_min || _state.get("y") < y_min);
    bool reset_condition = reset_condition_forward_x || reset_condition_backward_x;
    if (reset_condition)
    {
        _forward_x_experiment = !_forward_x_experiment;
    }
    return reset_condition;
}

void DingoPlanner::loop(const ros::TimerEvent &event)
{
    (void)event;
    LOG_MARK("============= Loop =============");
    _data.planning_start_time = std::chrono::system_clock::now();

    if (objectiveReached())
        reset();

    // Print the state
    if (CONFIG["debug_output"].as<bool>())
        _state.print();

    _benchmarker->start();

    auto output = _planner->solveMPC(_state, _data);

    LOG_VALUE_DEBUG("Success", output.success);

    geometry_msgs::Twist cmd;
    if (_enable_output && output.success)
    {
        LOG_MARK("Publishing command");

        // Output of MPC (vx, vy)
        // double output_angle =

        // Publish the command
        Eigen::Vector2d v(_planner->getSolution(1, "vx"), _planner->getSolution(1, "vy"));
        Eigen::Vector2d v_local = RosTools::rotationMatrixFromHeading(_measured_psi) * v;

        cmd.linear.x = v_local(0); //_planner->getSolution(1, "vx"); // = x1
        cmd.linear.y = v_local(1); //_planner->getSolution(1, "vy"); // = x1
        cmd.angular.z = 0.;
        _state.set("vx", v(0));
        _state.set("vy", v(1));
        LOG_VALUE_DEBUG("Commanded vx", cmd.linear.x);
        LOG_VALUE_DEBUG("Commanded vy", cmd.linear.y);
        CONFIG["enable_output"] = true;
    }
    else if (!_enable_output)
    {
        _state.set("vx", 0.);
        _state.set("vy", 0.);
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
    }
    else
    {

        cmd.linear.x = 0.;
        cmd.linear.y = 0.;
        cmd.angular.z = 0.;
        // _state.set("vx", v(0));
        // _state.set("vy", v(1));
        // LOG_VALUE_DEBUG("Commanded vx", cmd.linear.x);
        // LOG_VALUE_DEBUG("Commanded vy", cmd.linear.y);
        // CONFIG["enable_output"] = true;
    }
    _cmd_pub.publish(cmd);
    _benchmarker->stop();

    if (CONFIG["recording"]["enable"].as<bool>())
        _planner->saveData(_state, _data);

    _planner->visualize(_state, _data);
    visualize();

    LOG_DEBUG("============= End Loop =============");
}

void DingoPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);

    _state.set("vx", msg->twist.twist.linear.x);
    _state.set("vy", msg->twist.twist.linear.y);

    _measured_psi = RosTools::quaternionToAngle(msg->pose.pose.orientation); // The robot velocity command is in this direction

    // _measured_velocity = std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.));
    // _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));
}

void DingoPlanner::statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _state.set("x", msg->pose.position.x);
    _state.set("y", msg->pose.position.y);
    // _state.set("psi", msg->pose.orientation.z);
    // _measured_velocity = msg->pose.position.z;

    // _state.set("v", msg->pose.position.z);
}

void DingoPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    LOG_WARN("Goal callback");
    _data.goal(0) = msg->pose.position.x;
    _data.goal(1) = msg->pose.position.y;
    _data.goal_received = true;

    _planner->onDataReceived(_data, "goal");
}

bool DingoPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg)
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

void DingoPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
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

void DingoPlanner::obstacleCallback(const derived_object_msgs::ObjectArray::ConstPtr &msg)
{
    _data.dynamic_obstacles.clear();

    std::vector<double> angles;
    std::vector<Eigen::Vector2d> positions;
    std::vector<double> radii;
    std::vector<Eigen::Vector2d> twists;
    std::vector<ObstacleType> types;

    for (auto &object : msg->objects)
    {
        if (object.id == 0)
            continue;
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

    for (size_t i = 0; i < positions.size(); i++)
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

    ensureObstacleSize(_data.dynamic_obstacles, _state);
    propagatePredictionUncertainty(_data.dynamic_obstacles);

    _planner->onDataReceived(_data, "dynamic obstacles");
}

void DingoPlanner::parseObstacle(const derived_object_msgs::Object &object, double object_angle,
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

void DingoPlanner::bluetoothCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    if (msg->axes[2] < -0.9 && !_enable_output)
        LOG_INFO("Planning enabled (deadman switch pressed)");
    else if (msg->axes[2] > -0.9 && _enable_output)
        LOG_INFO("Deadmanswitch enabled (deadman switch released)");

    _enable_output = msg->axes[2] < -0.9;
    CONFIG["enable_output"] = _enable_output;
}

void DingoPlanner::visualize()
{
    return;
    auto &publisher = VISUALS.getPublisher("limits");
    auto &line = publisher.getNewLine();
    line.setColor(0., 0., 0.);
    line.setScale(0.1, 0.1);

    // Publish lab limits
    line.addLine(Eigen::Vector2d(x_min, y_min), Eigen::Vector2d(x_max, y_min));
    line.addLine(Eigen::Vector2d(x_max, y_min), Eigen::Vector2d(x_max, y_max));
    line.addLine(Eigen::Vector2d(x_max, y_max), Eigen::Vector2d(x_min, y_max));
    line.addLine(Eigen::Vector2d(x_min, y_max), Eigen::Vector2d(x_min, y_min));

    publisher.publish();

    auto &goal_publisher = VISUALS.getPublisher("goal");
    auto &cube = goal_publisher.getNewPointMarker("CUBE");

    cube.setScale(0.05, 0.05, 0.2);
    cube.setColorInt(4, 5);

    cube.addPointMarker(Eigen::Vector3d(_data.goal(0), _data.goal(1), 0.0));
    goal_publisher.publish();
}

void DingoPlanner::reset()
{
    LOG_INFO("Resetting");

    std_msgs::Empty empty_msg;
    _reverse_roadmap_pub.publish(empty_msg);

    _planner->reset(_state, _data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    ros::NodeHandle nh;
    auto dingo_planner = std::make_shared<DingoPlanner>(nh);
    VISUALS.init(&nh);

    ros::spin();

    return 0;
}