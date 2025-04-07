#include "robot_planning/yaml_common.h"
#include <ros/ros.h>

namespace robot_planning
{
    YAML::Node loadYAML(const std::string& filename)
    {
        try
        {
            return YAML::LoadFile(filename);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("Failed to load YAML file: " << filename << ". Error: " << e.what());
            return YAML::Node();
        }
    }

    Eigen::Matrix3f quaternionToRotationMatrix(float x, float y, float z, float w)
    {
        Eigen::Quaternionf q(w, x, y, z);
        return q.toRotationMatrix();
    }
}
