#pragma once

#include "IO.h"
#include "yaml-cpp/yaml.h"
#include <Eigen/Eigen>
#include <fmt/format.h>
#include "aero.h"

void initOutput();
void writeDescription();
void addDescription(YAML::Node &root);
void plot(const Aero& data, int ord);