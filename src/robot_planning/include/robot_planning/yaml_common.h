#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace robot_planning
{
    // 声明
    YAML::Node loadYAML(const std::string& filename);
    Eigen::Matrix3f quaternionToRotationMatrix(float x, float y, float z, float w);
}
