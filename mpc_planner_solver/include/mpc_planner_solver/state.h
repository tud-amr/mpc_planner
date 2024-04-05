#ifndef STATE_H
#define STATE_H

#include <mpc_planner_util/load_yaml.hpp>

#include <Eigen/Dense>

#include <vector>
#include <string>

namespace MPCPlanner
{
    struct State
    {
        State();

        void initialize();

        double get(std::string &&var_name) const;
        Eigen::Vector2d getPos() const;

        void set(std::string &&var_name, double value);
        void print() const;

    private:
        std::vector<double> _state;
        YAML::Node _config, _model_map;

        int _nu;
    };
}

#endif // STATE_H