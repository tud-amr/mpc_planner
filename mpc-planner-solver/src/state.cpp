#include "mpc-planner-solver/state.h"

#include <ros_planner_utils/logging.h>

using namespace MPCPlanner;

State::State()
{
    loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "solver_settings"), _config);
    loadConfigYaml(SYSTEM_CONFIG_PATH(__FILE__, "model_map"), _model_map);
    initialize();
}

void State::initialize()
{
    _state = std::vector<double>(_config["nx"].as<int>(), 0.0);
    _nu = _config["nu"].as<int>();
}

double State::get(std::string &&var_name) const
{
    return _state[_model_map[var_name][1].as<int>() - _nu]; // States come after the inputs
}

Eigen::Vector2d State::getPos() const
{
    return Eigen::Vector2d(get("x"), get("y"));
}

void State::set(std::string &&var_name, double value)
{
    _state[_model_map[var_name][1].as<int>() - _nu] = value;
}

void State::print() const
{
    for (YAML::const_iterator it = _model_map.begin(); it != _model_map.end(); ++it)
    {
        if (it->second[0].as<std::string>() == "x")
        {
            LOG_VALUE_DEBUG(it->first.as<std::string>(), get(it->first.as<std::string>()));
        }
    }
}