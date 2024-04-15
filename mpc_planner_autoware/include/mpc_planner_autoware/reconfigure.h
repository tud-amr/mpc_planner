#ifndef MPC_PLANNER_AUTOWARE_RECONFIGURE_H
#define MPC_PLANNER_AUTOWARE_RECONFIGURE_H

#include <mpc_planner_autoware/autoware_ros2_reconfigure.h>

class Reconfigure : public AutowareReconfigure
{
public:
    Reconfigure(rclcpp::Node *node)
        : AutowareReconfigure(node)
    {
        LOG_INFO("Setting up dynamic_reconfigure parameters");

        declareROSParameters(node);

        _set_param_res = node->add_on_set_parameters_callback(
            std::bind(&Reconfigure::updateROSParameters, this, std::placeholders::_1));
    }

    // Add your customizations here
    void declareROSParameters(rclcpp::Node *node) override
    {
        (void)node;

        node->declare_parameter<double>("road_width", CONFIG["road"]["width"].as<double>());
        node->declare_parameter<bool>("use_simulated_obstacles", CONFIG["use_simulated_obstacles"].as<bool>());

    }

    rcl_interfaces::msg::SetParametersResult updateROSParameters(const std::vector<rclcpp::Parameter> &parameters)
    {
        AutowareReconfigure::updateROSParameters(parameters);

        updateParam<double>(parameters, "road_width", CONFIG["road"]["width"]);
        updateParam<bool>(parameters, "use_simulated_obstacles", CONFIG["use_simulated_obstacles"]);

        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        return result;
    }

    // Readable actions

private:
};

#endif // MPC_PLANNER_AUTOWARE_RECONFIGURE_H
