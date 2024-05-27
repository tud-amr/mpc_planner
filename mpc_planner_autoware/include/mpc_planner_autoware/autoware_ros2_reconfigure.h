#ifndef __GENERATED_ROS2_RECONFIGURE_H
#define __GENERATED_ROS2_RECONFIGURE_H
#include <rclcpp/rclcpp.hpp>
#include <ros_tools/logging.h>
#include <ros_tools/ros2_wrappers.h>
#include <mpc_planner_util/parameters.h>
template <class T>
bool updateParam(const std::vector<rclcpp::Parameter> &params, const std::string &name, YAML::Node value)
{
	const auto itr = std::find_if(
		params.cbegin(), params.cend(),
		[&name](const rclcpp::Parameter &p)
		{ return p.get_name() == name; });

	// Not found
	if (itr == params.cend())
	{
		return false;
	}

	value = itr->template get_value<T>();
	LOG_INFO("Parameter " + name + " set to " + std::to_string(value.as<T>()));
	return true;
}

class AutowareReconfigure
{
public:
	AutowareReconfigure(rclcpp::Node *node)
	{
		LOG_INFO("Setting up dynamic_reconfigure parameters");

		declareROSParameters(node);

		_set_param_res = node->add_on_set_parameters_callback(
			std::bind(&AutowareReconfigure::updateROSParameters, this, std::placeholders::_1));
	}

	virtual void declareROSParameters(rclcpp::Node *node)
	{
		node->declare_parameter<double>("acceleration", CONFIG["weights"]["acceleration"].as<double>());
		node->declare_parameter<double>("angular_velocity", CONFIG["weights"]["angular_velocity"].as<double>());
		node->declare_parameter<double>("slack", CONFIG["weights"]["slack"].as<double>());
		node->declare_parameter<double>("contour", CONFIG["weights"]["contour"].as<double>());
		node->declare_parameter<double>("reference_velocity", CONFIG["weights"]["reference_velocity"].as<double>());
		node->declare_parameter<double>("velocity", CONFIG["weights"]["velocity"].as<double>());
		node->declare_parameter<double>("lag", CONFIG["weights"]["lag"].as<double>());
		node->declare_parameter<double>("terminal_angle", CONFIG["weights"]["terminal_angle"].as<double>());
		node->declare_parameter<double>("terminal_contouring", CONFIG["weights"]["terminal_contouring"].as<double>());
	}

	virtual rcl_interfaces::msg::SetParametersResult updateROSParameters(const std::vector<rclcpp::Parameter> &parameters)
	{
		updateParam<double>(parameters, "acceleration", CONFIG["weights"]["acceleration"]);
		updateParam<double>(parameters, "angular_velocity", CONFIG["weights"]["angular_velocity"]);
		updateParam<double>(parameters, "slack", CONFIG["weights"]["slack"]);
		updateParam<double>(parameters, "contour", CONFIG["weights"]["contour"]);
		updateParam<double>(parameters, "reference_velocity", CONFIG["weights"]["reference_velocity"]);
		updateParam<double>(parameters, "velocity", CONFIG["weights"]["velocity"]);
		updateParam<double>(parameters, "lag", CONFIG["weights"]["lag"]);
		updateParam<double>(parameters, "terminal_angle", CONFIG["weights"]["terminal_angle"]);
		updateParam<double>(parameters, "terminal_contouring", CONFIG["weights"]["terminal_contouring"]);

		auto result = rcl_interfaces::msg::SetParametersResult();
		result.successful = true;


		return result;
	}

protected:
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _set_param_res;
};
#endif // __GENERATED_ROS2_RECONFIGURE_H
