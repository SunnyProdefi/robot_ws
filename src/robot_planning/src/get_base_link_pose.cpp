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

bool calculateBaseLinkPose(robot_control::GetBaseLinkPose::Request &req, robot_control::GetBaseLinkPose::Response &res)
{
    robots::Kinematics kinematics;

    // 1. 计算 branch1 的正向运动学
    std::vector<float> joint_config_1;
    for (double val : req.joint_angles_branch1) joint_config_1.push_back(static_cast<float>(val));

    std::vector<float> ee_pose_1 = kinematics.forward(joint_config_1);

    if (ee_pose_1.size() != 12)
    {
        ROS_ERROR("Invalid FK result for branch1");
        return false;
    }

    Eigen::Matrix4f T_link1_0_flan1 = Eigen::Matrix4f::Identity();
    T_link1_0_flan1.block<3, 3>(0, 0) << ee_pose_1[0], ee_pose_1[1], ee_pose_1[2], ee_pose_1[4], ee_pose_1[5], ee_pose_1[6], ee_pose_1[8], ee_pose_1[9], ee_pose_1[10];
    T_link1_0_flan1(0, 3) = ee_pose_1[3];
    T_link1_0_flan1(1, 3) = ee_pose_1[7];
    T_link1_0_flan1(2, 3) = ee_pose_1[11];

    // 2. 计算 branch4 的正向运动学
    std::vector<float> joint_config_4;
    for (double val : req.joint_angles_branch4) joint_config_4.push_back(static_cast<float>(val));

    std::vector<float> ee_pose_4 = kinematics.forward(joint_config_4);

    if (ee_pose_4.size() != 12)
    {
        ROS_ERROR("Invalid FK result for branch4");
        return false;
    }

    Eigen::Matrix4f T_link4_0_flan4 = Eigen::Matrix4f::Identity();
    T_link4_0_flan4.block<3, 3>(0, 0) << ee_pose_4[0], ee_pose_4[1], ee_pose_4[2], ee_pose_4[3], ee_pose_4[4], ee_pose_4[5], ee_pose_4[6], ee_pose_4[7], ee_pose_4[8];
    T_link4_0_flan4(0, 3) = ee_pose_4[9];
    T_link4_0_flan4(1, 3) = ee_pose_4[10];
    T_link4_0_flan4(2, 3) = ee_pose_4[11];

    // 3. 计算 base_link → world（两种方式）
    Eigen::Matrix4f T_world_base1 = T_world_flan1 * (T_base_link1_0 * T_link1_0_flan1).inverse();
    Eigen::Matrix4f T_world_base4 = T_world_flan4 * (T_base_link4_0 * T_link4_0_flan4).inverse();

    // 4. 平均位置与旋转（位置平均 + 旋转Slerp或取一方）
    Eigen::Vector3f pos1 = T_world_base1.block<3, 1>(0, 3);
    Eigen::Vector3f pos4 = T_world_base4.block<3, 1>(0, 3);
    Eigen::Vector3f avg_pos = 0.5 * (pos1 + pos4);

    Eigen::Matrix3f rot1 = T_world_base1.block<3, 3>(0, 0);
    Eigen::Matrix3f rot4 = T_world_base4.block<3, 3>(0, 0);
    Eigen::Quaternionf q1(rot1);
    Eigen::Quaternionf q4(rot4);
    Eigen::Quaternionf avg_q = q1.slerp(0.5, q4);  // 球形插值

    q1.normalize();  // 确保四元数单位化

    // 5. 填充响应
    res.base_link_pose.position.x = pos1.x();
    res.base_link_pose.position.y = pos1.y();
    res.base_link_pose.position.z = pos1.z();

    res.base_link_pose.orientation.x = q1.x();
    res.base_link_pose.orientation.y = q1.y();
    res.base_link_pose.orientation.z = q1.z();
    res.base_link_pose.orientation.w = q1.w();

    ROS_INFO("Computed world → base_link transform using dual-branch kinematics.");

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_base_link_pose");
    ros::NodeHandle nh;

    // 创建服务
    ros::ServiceServer service = nh.advertiseService("get_base_link_pose", calculateBaseLinkPose);
    ROS_INFO("Service ready to calculate base_link pose.");

    // 1. 读取 TF 配置文件
    std::string package_path = ros::package::getPath("robot_planning");
    std::string tf_using_file = package_path + "/config/tf_using.yaml";
    YAML::Node tf_yaml = YAML::LoadFile(tf_using_file);

    // 2. 加载静态变换矩阵
    T_world_flan1 = parseTransformMatrix(tf_yaml["tf_mat_world_flan1"]);
    T_world_flan4 = parseTransformMatrix(tf_yaml["tf_mat_world_flan4"]);
    T_base_link1_0 = parseTransformMatrix(tf_yaml["tf_mat_base_link1_0"]);
    T_base_link4_0 = parseTransformMatrix(tf_yaml["tf_mat_base_link4_0"]);

    ros::spin();
    return 0;
}
