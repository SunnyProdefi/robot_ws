#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <robot_control/GetBaseLinkPose.h>
#include <Eigen/Dense>
#include "robot_planning/ikfast_wrapper_single_arm.h"
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

Eigen::Matrix4f T_world_flan1;
Eigen::Matrix4f T_world_flan4;
Eigen::Matrix4f T_base_link1_0;
Eigen::Matrix4f T_base_link4_0;

robots::Kinematics kinematics;

Eigen::Matrix4f parseTransformMatrix(const YAML::Node &node)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    auto pos = node["target_pose"]["position"];
    auto ori = node["target_pose"]["orientation"];

    // 设置平移
    T(0, 3) = pos[0].as<float>();
    T(1, 3) = pos[1].as<float>();
    T(2, 3) = pos[2].as<float>();

    // 设置旋转矩阵
    for (int i = 0; i < 3; ++i)
    {
        auto row = ori[i];
        for (int j = 0; j < 3; ++j)
        {
            T(i, j) = row[j].as<float>();
        }
    }

    return T;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_base_link_pose");
    ros::NodeHandle nh;

    // 1. 读取 TF 配置文件
    std::string package_path = ros::package::getPath("robot_planning");
    std::string tf_using_file = package_path + "/config/tf_using.yaml";
    YAML::Node tf_yaml = YAML::LoadFile(tf_using_file);

    // 2. 加载静态变换矩阵
    T_world_flan1 = parseTransformMatrix(tf_yaml["tf_mat_world_flan1"]);
    T_world_flan4 = parseTransformMatrix(tf_yaml["tf_mat_world_flan4"]);
    T_base_link1_0 = parseTransformMatrix(tf_yaml["tf_mat_base_link1_0"]);
    T_base_link4_0 = parseTransformMatrix(tf_yaml["tf_mat_base_link4_0"]);

    // 1. 计算 branch1 的正向运动学
    std::vector<float> joint_config_1 = {1.597743, 0.2950242, 2.156446, 3.101645, -0.4948243, -0.01648967};  // 示例配置
    std::vector<float> ee_pose_1 = kinematics.forward(joint_config_1);

    if (ee_pose_1.size() != 12)
    {
        ROS_ERROR("Invalid FK result for branch1");
        return false;
    }

    // 这里有问题
    Eigen::Matrix4f T_link1_0_flan1 = Eigen::Matrix4f::Identity();
    T_link1_0_flan1.block<3, 3>(0, 0) << ee_pose_1[0], ee_pose_1[1], ee_pose_1[2], ee_pose_1[3], ee_pose_1[4], ee_pose_1[5], ee_pose_1[6], ee_pose_1[7], ee_pose_1[8];
    T_link1_0_flan1(0, 3) = ee_pose_1[9];
    T_link1_0_flan1(1, 3) = ee_pose_1[10];
    T_link1_0_flan1(2, 3) = ee_pose_1[11];

    // 打印T_link1_0_flan1
    std::cout << "T_link1_0_flan1:" << std::endl;
    std::cout << T_link1_0_flan1 << std::endl;

    // 3. 计算 base_link → world（两种方式）
    Eigen::Matrix4f T_world_base1 = T_world_flan1 * (T_base_link1_0 * T_link1_0_flan1).inverse();

    // 4. 平均位置与旋转（位置平均 + 旋转Slerp或取一方）
    Eigen::Vector3f pos1 = T_world_base1.block<3, 1>(0, 3);

    Eigen::Matrix3f rot1 = T_world_base1.block<3, 3>(0, 0);
    Eigen::Quaternionf q1(rot1);

    q1.normalize();  // 确保四元数单位化

    ros::spin();
    return 0;
}
