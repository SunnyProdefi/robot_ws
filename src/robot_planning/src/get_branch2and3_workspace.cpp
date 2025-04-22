#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <random>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include "robot_planning/ikfast_wrapper_single_arm.h"

// 转换矩阵 → 向量
std::vector<float> transformToVector(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans)
{
    std::vector<float> pose(12);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j) pose[i * 4 + j] = rot(i, j);
        pose[i * 4 + 3] = trans(i);
    }
    return pose;
}

// 读取 tf pose from YAML
void readTransformFromYAML(const YAML::Node& node, Eigen::Matrix4d& transform)
{
    auto position = node["target_pose"]["position"].as<std::vector<double>>();
    auto orientation = node["target_pose"]["orientation"];
    transform.setIdentity();
    for (int i = 0; i < 3; ++i)
    {
        auto row = orientation[i].as<std::vector<double>>();
        for (int j = 0; j < 3; ++j) transform(i, j) = row[j];
        transform(i, 3) = position[i];
    }
}

// 保存末端工作空间到 YAML
void saveWorkspaceToYAML(const std::vector<Eigen::Matrix4d>& poses, const std::string& filename)
{
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        const auto& T = poses[i];
        Eigen::Vector3d t = T.block<3, 1>(0, 3);
        Eigen::Quaterniond q(T.block<3, 3>(0, 0));
        out << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << static_cast<int>(i + 1);
        out << YAML::Key << "position" << YAML::Value << YAML::Flow << std::vector<double>{t.x(), t.y(), t.z()};
        out << YAML::Key << "orientation" << YAML::Value << YAML::Flow << std::vector<double>{q.w(), q.x(), q.y(), q.z()};
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    std::ofstream fout(filename);
    fout << out.c_str();
    ROS_INFO("Saved workspace to %s", filename.c_str());
}

void computeWorkspace(int branch_id, const Eigen::Matrix4d& T_base_link, const std::string& output_path, robots::Kinematics& ik_solver)
{
    std::vector<std::pair<float, float>> joint_limits = {{-2.3562f, 4.7124f}, {-2.0944f, 1.5708f}, {-2.5307f, 2.5307f}, {-3.1416f, 3.1416f}, {-1.9199f, 1.9199f}, {-3.1416f, 3.1416f}};

    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<std::uniform_real_distribution<>> distributions;
    for (const auto& lim : joint_limits) distributions.emplace_back(lim.first, lim.second);

    const int NUM_SAMPLES = 1000000;
    std::vector<Eigen::Matrix4d> workspace_poses;

    for (int i = 0; i < NUM_SAMPLES; ++i)
    {
        std::vector<float> joint_config;
        for (int j = 0; j < 6; ++j) joint_config.push_back(distributions[j](gen));

        std::vector<float> ee_pose = ik_solver.forward(joint_config);
        if (ee_pose.empty())
            continue;

        Eigen::Matrix4d T_ee = Eigen::Matrix4d::Identity();
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c) T_ee(r, c) = ee_pose[r * 4 + c];
            T_ee(r, 3) = ee_pose[r * 4 + 3];
        }

        Eigen::Matrix4d T_world_ee = T_base_link * T_ee;
        workspace_poses.push_back(T_world_ee);
    }

    saveWorkspaceToYAML(workspace_poses, output_path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "compute_workspace_branch2_3");

    YAML::Node config = YAML::LoadFile("/home/prodefi/github/robot_ws/src/robot_planning/config/workspace.yaml");

    Eigen::Matrix4d T_base_link2_0, T_base_link3_0;
    readTransformFromYAML(config["tf_mat_base_link2_0"], T_base_link2_0);
    readTransformFromYAML(config["tf_mat_base_link3_0"], T_base_link3_0);

    // 使用同一个求解器类，如果你分支2/3有不同模型，请分别创建
    robots::Kinematics ik_solver_branch2, ik_solver_branch3;

    computeWorkspace(2, T_base_link2_0, "/home/prodefi/github/robot_ws/src/robot_planning/config/workspace_branch2.yaml", ik_solver_branch2);
    computeWorkspace(3, T_base_link3_0, "/home/prodefi/github/robot_ws/src/robot_planning/config/workspace_branch3.yaml", ik_solver_branch3);

    return 0;
}
