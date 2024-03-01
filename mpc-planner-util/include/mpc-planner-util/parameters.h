#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <mpc-planner-util/load_yaml.hpp>

#define CONFIG Configuration::getInstance().getYAMLNode()

namespace YAML
{
    class SafeNode
    {
        template <typename Key>
        const Node operator[](const Key &key) const;
    };

}

class Configuration
{
public:
    static Configuration &getInstance()
    {
        static Configuration instance;
        return instance;
    }

    void initialize(const std::string &config_file)
    {
        loadConfigYaml(config_file, _config); // Load parameters from the YAML file
    }

    YAML::Node &getYAMLNode()
    {
        return _config;
    }

private:
    YAML::Node _config;

    Configuration()
    {
        // Initialize your configuration variables here
    }

    Configuration(const Configuration &) = delete;
    Configuration &operator=(const Configuration &) = delete;
};

#endif // PARAMETERS_H
