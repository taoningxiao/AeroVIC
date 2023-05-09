#pragma once
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#define Vec2d Eigen::Vector2d
#define Vec2i Eigen::Vector2i

extern YAML::Node config;
void load_config(std::string config_file);