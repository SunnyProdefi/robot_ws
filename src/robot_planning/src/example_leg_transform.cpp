#include <ros/ros.h>
#include "robot_planning/leg_transform_cs.h"
#include <ros/package.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_leg_transform");
    ros::NodeHandle nh;

    // 获取包路径
    std::string package_path = ros::package::getPath("robot_planning");
    
    // 设置文件路径
    std::string init_floating_base_file = package_path + "/config/ik_urdf_double_arm_float.yaml";
    std::string gold_floating_base_file = package_path + "/config/planned_trajectory.yaml";
    std::string tf_using_file = package_path + "/config/tf_using.yaml";
    std::string output_file = package_path + "/config/leg_ik_cs.yaml";

    // 调用计算函数
    bool success = robot_planning::computeLegTransforms(
        init_floating_base_file,
        gold_floating_base_file,
        tf_using_file,
        output_file);

    if (success)
    {
        ROS_INFO("Leg transform computation completed successfully!");
    }
    else
    {
        ROS_ERROR("Failed to compute leg transforms!");
        return -1;
    }

    return 0;
} 