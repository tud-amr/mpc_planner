#ifndef __GENERATED_RECONFIGURE_H
#define __GENERATED_RECONFIGURE_H

#include <ros/ros.h>

#include <ros_tools/logging.h>
#include <mpc_planner_util/parameters.h>

// Dynamic Reconfigure server
#include <dynamic_reconfigure/server.h>
#include <mpc_planner_autoware/Config.h>

class AutowareReconfigure
{
public:
	AutowareReconfigure()
	{
		// Initialize the dynamic reconfiguration
		LOG_INFO("Setting up dynamic_reconfigure server for the parameters");
		// first_reconfigure_callback_ = true;
		ros::NodeHandle nh_reconfigure("mpc_planner_autoware");
		_reconfigure_server.reset(new dynamic_reconfigure::Server<mpc_planner_autoware::Config>(_reconfig_mutex, nh_reconfigure));
		_reconfigure_server->setCallback(boost::bind(&AutowareReconfigure::reconfigureCallback, this, _1, _2));
	}
	void reconfigureCallback(mpc_planner_autoware::Config &config, uint32_t level)
	{
		(void)level;
		if (_first_reconfigure_callback){
			config.acceleration = CONFIG["weights"]["acceleration"].as<double>();
			config.angular_velocity = CONFIG["weights"]["angular_velocity"].as<double>();
			config.slack = CONFIG["weights"]["slack"].as<double>();
			config.contour = CONFIG["weights"]["contour"].as<double>();
			config.reference_velocity = CONFIG["weights"]["reference_velocity"].as<double>();
			config.velocity = CONFIG["weights"]["velocity"].as<double>();
			config.lag = CONFIG["weights"]["lag"].as<double>();
			config.preview = CONFIG["weights"]["preview"].as<double>();
			_first_reconfigure_callback = false;
		}else{
			CONFIG["weights"]["acceleration"] = config.acceleration;
			CONFIG["weights"]["angular_velocity"] = config.angular_velocity;
			CONFIG["weights"]["slack"] = config.slack;
			CONFIG["weights"]["contour"] = config.contour;
			CONFIG["weights"]["reference_velocity"] = config.reference_velocity;
			CONFIG["weights"]["velocity"] = config.velocity;
			CONFIG["weights"]["lag"] = config.lag;
			CONFIG["weights"]["preview"] = config.preview;
		}
	}

private:
	bool _first_reconfigure_callback{true};
	// RQT Reconfigure ROS1
	boost::shared_ptr<dynamic_reconfigure::Server<mpc_planner_autoware::Config>> _reconfigure_server;
	boost::recursive_mutex _reconfig_mutex;
};

#endif // __GENERATED_RECONFIGURE_H
