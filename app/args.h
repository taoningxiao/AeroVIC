#pragma once
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#define Vec2d Eigen::Vector2d
#define Vec2i Eigen::Vector2i

// parallel macro
// #define USE_TBB 1
#define USE_OPENMP 1

const std::string monitor_func[] = {
    "total",
    "simulate",
    "simulate/advect", 
    "simulate/buildGridTable", 
    "simulate/vortexP2G", 
    "simulate/diffusion", 
    "simulate/calVelocity", 
    "simulate/boundaryPenalty", 
    "simulate/calVortex",
    "simulate/vortexG2P", 
    "plot"
};

const double EPS = 1e-6;

extern YAML::Node config;
void load_config(std::string config_file);