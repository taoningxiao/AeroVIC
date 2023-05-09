#include "args.h"
#include "yaml-cpp/node/node.h"

YAML::Node config;

void load_config(std::string config_file) {
    config = YAML::LoadFile(config_file);
}