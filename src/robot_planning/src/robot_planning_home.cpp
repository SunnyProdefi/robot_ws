#include <ros/ros.h>
#include "robot_planning/PlanPathHome.h"
#include "robot_planning/leg_transform_cs.h"
#include "robot_planning/leg_ik_cs.h"
#include "robot_planning/full_body_trajectory.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <Eigen/Dense>
#include <ros/package.h>

int TOTAL_POINT_NUM = 600;
int POINT_NUM = TOTAL_POINT_NUM / 4;

// 将 SE(3) 位姿数组 [x, y, z, qx, qy, qz, qw] 转换为 4x4 矩阵
Eigen::Matrix4f vectorToMatrix(const std::vector<double>& pose_vec)
{
    Eigen::Vector3f t(pose_vec[0], pose_vec[1], pose_vec[2]);
    Eigen::Quaternionf q(pose_vec[6], pose_vec[3], pose_vec[4], pose_vec[5]);  // qw, qx, qy, qz
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    T.block<3, 1>(0, 3) = t;
    return T;
}

void savePoseAndJointsToYAML(const std::string& file_path, const std::string& pose_key, const std::vector<double>& pose, const std::string& joints_key, const std::vector<double>& joints)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << pose_key << YAML::Value << pose;
    out << YAML::Key << joints_key << YAML::Value << joints;
    out << YAML::EndMap;

    std::ofstream fout(file_path);
    fout << out.c_str();
    fout.close();
}

bool planCallback(robot_planning::PlanPathHome::Request& req, robot_planning::PlanPathHome::Response& res)
{
    ROS_INFO("Received planning request...");

    std::string package_path = ros::package::getPath("robot_planning");

    // 构建文件路径
    std::string init_config_file = package_path + "/config/init_config.yaml";
    std::string gold_config_file = package_path + "/config/gold_config.yaml";

    // 保存 init 数据
    savePoseAndJointsToYAML(init_config_file, "init_floating_base", req.init_floating_base, "init_joint_angles", req.init_joint_angles);

    // 保存 gold 数据
    savePoseAndJointsToYAML(gold_config_file, "gold_floating_base", req.gold_floating_base, "gold_joint_angles", req.gold_joint_angles);

    std::string tf_using_file = package_path + "/config/tf_using.yaml";
    std::string output_file = package_path + "/config/leg_ik_cs_home.yaml";
    std::string result_cs_file = package_path + "/config/result_cs_home.yaml";
    std::string planning_result_file = package_path + "/config/planning_result_home.yaml";

    // Step 1: 计算腿末端目标变换
    if (!robot_planning::computeLegTransforms_home(init_config_file, gold_config_file, tf_using_file, output_file))
    {
        res.success = false;
        res.message = "Failed to compute leg transforms!";
        return true;
    }

    // Step 2: 腿部IK求解
    auto leg_ik_result = robot_planning::solveLegIK(output_file, result_cs_file, "tf_mat_link1_0_flan1", "tf_mat_link4_0_flan4");
    if (!leg_ik_result.success)
    {
        res.success = false;
        res.message = "Failed to compute leg IK: " + leg_ik_result.message;
        return true;
    }

    // 整合4个分支与float_base的结果
    std::vector<std::vector<double>> joint_angles(TOTAL_POINT_NUM, std::vector<double>(24, 0.0));
    // 例如 1000 帧，每帧有 TOTAL_JOINT_NUM 个关节

    robot_planning::loadJointAngles(init_config_file, gold_config_file, result_cs_file, joint_angles);

    // 插值floating base
    std::vector<Eigen::Matrix4f> base_sequence = robot_planning::interpolateFloatingBase_home(init_config_file, gold_config_file, POINT_NUM);

    // 将结果写出
    robot_planning::saveFullBodyTrajectoryToYAML(planning_result_file, joint_angles, base_sequence);

    // 设置响应
    res.success = true;
    res.message = "Path planning completed successfully!";
    ROS_INFO("Path planning completed successfully!");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner_home");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("plan_path_home", planCallback);
    ROS_INFO("Path planner home node ready.");
    ros::spin();
    return 0;
}
