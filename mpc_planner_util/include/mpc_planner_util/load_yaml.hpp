#ifndef LOAD_YAML_H
#define LOAD_YAML_H

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>

#define SYSTEM_CONFIG_PATH(x, filename) std::filesystem::path(x).parent_path().string() + "/../config/" + filename + ".yaml"
#define SYSTEM_CONFIG_PATH_INCLUDE(x, filename) std::filesystem::path(x).parent_path().parent_path().string() + "/../config/" + filename + ".yaml"

inline void loadConfigYaml(const std::string &file, YAML::Node &_yaml_out)
{
    _yaml_out = YAML::LoadFile(file);
}

#endif // LOAD_YAML_H