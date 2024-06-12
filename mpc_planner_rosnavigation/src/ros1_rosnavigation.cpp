#include <mpc_planner_rosnavigation/ros1_rosnavigation.h>
#include <pluginlib/class_list_macros.h>

#include <mpc_planner/planner.h>

#include <mpc_planner/data_preparation.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>
#include <ros_tools/spline.h>

#include <std_msgs/Empty.h>
#include <ros_tools/profiling.h>

using namespace MPCPlanner;

PLUGINLIB_EXPORT_CLASS(local_planner::ROSNavigationPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{

    ROSNavigationPlanner::ROSNavigationPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

    ROSNavigationPlanner::ROSNavigationPlanner(std::string name, tf2_ros::Buffer *tf,
                                               costmap_2d::Costmap2DROS *costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }

    void ROSNavigationPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                          costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            ros::NodeHandle nh("~/" + name);

            tf_ = tf;

            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            _data.costmap = costmap_;

            initialized_ = true;

            LOG_INFO("Started ROSNavigation Planner");

            VISUALS.init(&general_nh_);

            // Initialize the configuration
            Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

            _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())};

            // Initialize the planner
            _planner = std::make_unique<Planner>();

            // Initialize the ROS interface
            initializeSubscribersAndPublishers(nh);

            startEnvironment();

            _reconfigure = std::make_unique<RosnavigationReconfigure>();

            _timeout_timer.setDuration(60.);
            _timeout_timer.start();
            for (int i = 0; i < CAMERA_BUFFER; i++)
            {
                _x_buffer[i] = 0.;
                _y_buffer[i] = 0.;
            }

            RosTools::Instrumentor::Get().BeginSession("mpc_planner_rosnavigation");

            LOG_DIVIDER();
        }
    }

    ROSNavigationPlanner::~ROSNavigationPlanner()
    {
        LOG_INFO("Stopped ROSNavigation Planner");
        BENCHMARKERS.print();

        RosTools::Instrumentor::Get().EndSession();
    }

    bool ROSNavigationPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        // check if plugin is initialized
        if (!initialized_)
        {
            ROS_ERROR("planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        // store the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;

        // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
        // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

        // reset goal_reached_ flag
        // goal_reached_ = false;

        return true;
    }

    bool ROSNavigationPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        auto path = boost::make_shared<nav_msgs::Path>();
        path->poses = global_plan_;
        pathCallback(path);

        if (_rotate_to_goal)
            rotateToGoal(cmd_vel);
        else
            loop(cmd_vel);

        return true;
    }

    void ROSNavigationPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
    {
        LOG_INFO("initializeSubscribersAndPublishers");

        _state_sub = nh.subscribe<nav_msgs::Odometry>(
            "/input/state", 5,
            boost::bind(&ROSNavigationPlanner::stateCallback, this, _1));

        _state_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            "/input/state_pose", 5,
            boost::bind(&ROSNavigationPlanner::statePoseCallback, this, _1));

        _goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            "/input/goal", 1,
            boost::bind(&ROSNavigationPlanner::goalCallback, this, _1));

        _path_sub = nh.subscribe<nav_msgs::Path>(
            "/input/reference_path", 1,
            boost::bind(&ROSNavigationPlanner::pathCallback, this, _1));

        _obstacle_sim_sub = nh.subscribe<mpc_planner_msgs::ObstacleArray>(
            "/input/obstacles", 1,
            boost::bind(&ROSNavigationPlanner::obstacleCallback, this, _1));

        _cmd_pub = nh.advertise<geometry_msgs::Twist>(
            "/output/command", 1);

        _pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
            "/output/pose", 1);

        _collisions_sub = nh.subscribe<std_msgs::Float64>(
            "/feedback/collisions", 1,
            boost::bind(&ROSNavigationPlanner::collisionCallback, this, _1));

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

    void ROSNavigationPlanner::startEnvironment()
    {

        // Manually add obstacles in the costmap!
        // int mx, my;
        // costmap_->worldToMapEnforceBounds(2., 2., mx, my);
        // LOG_VALUE("mx", mx);
        // LOG_VALUE("my", my);

        // for (int i = 0; i < 10; i++)
        // {
        //     costmap_->setCost(mx + i, my, costmap_2d::LETHAL_OBSTACLE);
        // }

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

                _reset_simulation_pub.publish(std_msgs::Empty());
            }
        }
        _enable_output = CONFIG["enable_output"].as<bool>();
        LOG_INFO("Environment ready.");
    }

    bool ROSNavigationPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        bool goal_reached = _planner->isObjectiveReached(_state, _data) && !done_; // Activate once
        if (goal_reached)
        {
            LOG_SUCCESS("Goal Reached!");
            done_ = true;
            reset();
        }

        return goal_reached;
    }

    void ROSNavigationPlanner::rotateToGoal(geometry_msgs::Twist &cmd_vel)
    {
        LOG_INFO_THROTTLE(1500, "Rotating to the goal");
        if (!_data.goal_received)
        {
            LOG_INFO("Waiting for the goal");
            return;
        }
        double goal_angle = 0.;

        if (_data.reference_path.x.size() > 2)
            goal_angle = std::atan2(_data.reference_path.y[2] - _state.get("y"), _data.reference_path.x[2] - _state.get("x"));
        else
            goal_angle = std::atan2(_data.goal(1) - _state.get("y"), _data.goal(0) - _state.get("x"));

        double angle_diff = goal_angle - _state.get("psi");

        if (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;

        geometry_msgs::Twist cmd;
        if (std::abs(angle_diff) > M_PI / 4.)
        {
            cmd_vel.linear.x = 0.0;
            if (_enable_output)
                cmd_vel.angular.z = 1.5 * RosTools::sgn(angle_diff);
            else
                cmd_vel.angular.z = 0.;
        }
        else
        {
            LOG_SUCCESS("Robot rotated and is ready to follow the path");
            _rotate_to_goal = false;
        }
    }

    void ROSNavigationPlanner::loop(geometry_msgs::Twist &cmd_vel)
    {

        // Copy data for thread safety
        RealTimeData data = _data;
        State state = _state;

        data.planning_start_time = std::chrono::system_clock::now();

        LOG_MARK("============= Loop =============");

        if (_timeout_timer.hasFinished())
        {
            reset(false);
            cmd_vel.linear.x = 0.;
            cmd_vel.angular.z = 0.;
            return;
        }

        if (CONFIG["debug_output"].as<bool>())
            state.print();

        auto &loop_benchmarker = BENCHMARKERS.getBenchmarker("loop");
        loop_benchmarker.start();

        auto output = _planner->solveMPC(state, data);

        LOG_MARK("Success: " << output.success);

        geometry_msgs::Twist cmd;
        if (_enable_output && output.success)
        {
            // Publish the command
            cmd_vel.linear.x = _planner->getSolution(1, "v");  // = x1
            cmd_vel.angular.z = _planner->getSolution(0, "w"); // = u0
            LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
            LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
        }
        else
        {
            double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
            double velocity_after_braking;
            double velocity;
            double dt = 1. / CONFIG["control_frequency"].as<double>();

            velocity = _state.get("v");
            velocity_after_braking = velocity - deceleration * dt;   // Brake with the given deceleration
            cmd_vel.linear.x = std::max(velocity_after_braking, 0.); // Don't drive backwards when braking
            cmd_vel.angular.z = 0.0;
        }
        _cmd_pub.publish(cmd);

        publishPose();
        publishCamera();

        loop_benchmarker.stop();

        if (CONFIG["recording"]["enable"].as<bool>())
        {

            // Save control inputs
            if (output.success)
            {
                auto &data_saver = _planner->getDataSaver();
                data_saver.AddData("input_a", state.get("a"));
                data_saver.AddData("input_v", _planner->getSolution(1, "v"));
                data_saver.AddData("input_w", _planner->getSolution(0, "w"));
            }

            _planner->saveData(state, data);
        }
        if (output.success)
        {
            _planner->visualize(state, data);
            visualize();
        }
        LOG_MARK("============= End Loop =============");
    }

    void ROSNavigationPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        LOG_MARK("State callback");
        _state.set("x", msg->pose.pose.position.x);
        _state.set("y", msg->pose.pose.position.y);
        _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
        _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));

        if (std::abs(msg->pose.pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.pose.orientation.y) > (M_PI / 8.))
        {
            LOG_WARN("Detected flipped robot. Resetting.");
            reset(false); // Reset without success
        }
    }

    void ROSNavigationPlanner::statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        LOG_MARK("State callback");

        _state.set("x", msg->pose.position.x);
        _state.set("y", msg->pose.position.y);
        _state.set("psi", msg->pose.orientation.z);
        _state.set("v", msg->pose.position.z);

        if (std::abs(msg->pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.orientation.y) > (M_PI / 8.))
        {
            LOG_ERROR("Detected flipped robot. Resetting.");
            reset(false); // Reset without success
        }
    }

    void ROSNavigationPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        LOG_MARK("Goal callback");

        _data.goal(0) = msg->pose.position.x;
        _data.goal(1) = msg->pose.position.y;
        _data.goal_received = true;

        _rotate_to_goal = true;
    }

    bool ROSNavigationPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg)
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

    void ROSNavigationPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        LOG_MARK("Path callback");

        int downsample = CONFIG["downsample_path"].as<double>();

        if (isPathTheSame(msg) || msg->poses.size() < downsample + 1)
            return;

        _data.reference_path.clear();

        int count = 0;
        for (auto &pose : msg->poses)
        {
            if (count % downsample == 0 || count == msg->poses.size() - 1) // Todo
            {
                _data.reference_path.x.push_back(pose.pose.position.x);
                _data.reference_path.y.push_back(pose.pose.position.y);
                _data.reference_path.psi.push_back(RosTools::quaternionToAngle(pose.pose.orientation));
            }
            count++;
        }

        // Fit a clothoid on the global path to sample points on the spline from
        // RosTools::Clothoid2D clothoid(_data.reference_path.x, _data.reference_path.y, _data.reference_path.psi, 2.0);
        // _data.reference_path.clear();
        // clothoid.getPointsOnClothoid(_data.reference_path.x, _data.reference_path.y, _data.reference_path.s);

        // Velocity
        /*LOG_VALUE("velocity reference", CONFIG["weights"]["reference_velocity"].as<double>());
        for (size_t i = 0; i < _data.reference_path.x.size(); i++)
        {
            if (i != _data.reference_path.x.size() - 1)
                _data.reference_path.v.push_back(CONFIG["weights"]["reference_velocity"].as<double>());
            else
                _data.reference_path.v.push_back(0.);
        }*/

        _planner->onDataReceived(_data, "reference_path");
    }

    void ROSNavigationPlanner::obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg)
    {
        LOG_MARK("Obstacle callback");

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
        ensureObstacleSize(_data.dynamic_obstacles, _state);

        if (CONFIG["probabilistic"]["propagate_uncertainty"].as<bool>())
            propagatePredictionUncertainty(_data.dynamic_obstacles);

        _planner->onDataReceived(_data, "dynamic obstacles");
    }

    void ROSNavigationPlanner::visualize()
    {
        auto &publisher = VISUALS.getPublisher("angle");
        auto &line = publisher.getNewLine();

        line.addLine(Eigen::Vector2d(_state.get("x"), _state.get("y")),
                     Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")), _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
        publisher.publish();
    }

    void ROSNavigationPlanner::reset(bool success)
    {
        LOG_INFO("Resetting");
        boost::mutex::scoped_lock l(_reset_mutex);

        _reset_simulation_client.call(_reset_msg);
        _reset_ekf_client.call(_reset_pose_msg);
        _reset_simulation_pub.publish(std_msgs::Empty());

        for (int i = 0; i < CAMERA_BUFFER; i++)
        {
            _x_buffer[i] = 0.;
            _y_buffer[i] = 0.;
        }

        _planner->reset(_state, _data, success);
        _data.costmap = costmap_;

        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()).sleep();

        done_ = false;
        _rotate_to_goal = false;

        _timeout_timer.start();
    }

    void ROSNavigationPlanner::collisionCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        LOG_MARK("Collision callback");

        _data.intrusion = (float)(msg->data);

        if (_data.intrusion > 0.)
            LOG_INFO_THROTTLE(500., "Collision detected (Intrusion: " << _data.intrusion << ")");
    }

    void ROSNavigationPlanner::publishPose()
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = _state.get("x");
        pose.pose.position.y = _state.get("y");
        pose.pose.orientation = RosTools::angleToQuaternion(_state.get("psi"));

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        _pose_pub.publish(pose);
    }

    void ROSNavigationPlanner::publishCamera()
    {
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();

        if ((msg.header.stamp - _prev_stamp) < ros::Duration(0.5 / CONFIG["control_frequency"].as<double>()))
            return;

        _prev_stamp = msg.header.stamp;

        msg.header.frame_id = "map";
        msg.child_frame_id = "camera";

        // Smoothen the camera
        for (int i = 0; i < CAMERA_BUFFER - 1; i++)
        {
            _x_buffer[i] = _x_buffer[i + 1];
            _y_buffer[i] = _y_buffer[i + 1];
        }
        _x_buffer[CAMERA_BUFFER - 1] = _state.get("x");
        _y_buffer[CAMERA_BUFFER - 1] = _state.get("y");
        double camera_x = 0., camera_y = 0.;
        for (int i = 0; i < CAMERA_BUFFER; i++)
        {
            camera_x += _x_buffer[i];
            camera_y += _y_buffer[i];
        }
        msg.transform.translation.x = camera_x / (double)CAMERA_BUFFER; //_state.get("x");
        msg.transform.translation.y = camera_y / (double)CAMERA_BUFFER; //_state.get("y");
        msg.transform.translation.z = 0.0;
        msg.transform.rotation.x = 0;
        msg.transform.rotation.y = 0;
        msg.transform.rotation.z = 0;
        msg.transform.rotation.w = 1;

        _camera_pub.sendTransform(msg);
    }

} // namespace local_planner