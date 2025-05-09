#include <ros/ros.h>
#include "robot_planning/PlanPath.h"
#include "robot_planning/ik_solver.h"
#include "robot_planning/leg_transform_cs.h"
#include "robot_planning/leg_ik_cs.h"
#include "robot_planning/full_body_trajectory.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <Eigen/Dense>
#include <ros/package.h>

int TOTAL_POINT_NUM = 600;
int POINT_NUM = TOTAL_POINT_NUM / 4;

bool planCallback(robot_planning::PlanPath::Request& req, robot_planning::PlanPath::Response& res)
{
    ROS_INFO("Received planning request...");

    // 获取包路径
    std::string package_path = ros::package::getPath("robot_planning");
    ROS_INFO("Package path: %s", package_path.c_str());

    // 构建文件路径
    std::string urdf_path = package_path + "/models/double_arms_description.urdf";
    std::string joint_limits_path = package_path + "/config/joint_limits.yaml";
    std::string init_config_path = package_path + "/config/ik_urdf_double_arm_float.yaml";

    // 检查文件是否存在
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.good())
    {
        ROS_ERROR("URDF file not found: %s", urdf_path.c_str());
        res.success = false;
        res.message = "URDF file not found: " + urdf_path;
        return true;
    }
    urdf_file.close();

    std::ifstream joint_limits_file(joint_limits_path);
    if (!joint_limits_file.good())
    {
        ROS_ERROR("Joint limits file not found: %s", joint_limits_path.c_str());
        res.success = false;
        res.message = "Joint limits file not found: " + joint_limits_path;
        return true;
    }
    joint_limits_file.close();

    std::ifstream init_config_file(init_config_path);
    if (!init_config_file.good())
    {
        ROS_ERROR("Initial config file not found: %s", init_config_path.c_str());
        res.success = false;
        res.message = "Initial config file not found: " + init_config_path;
        return true;
    }
    init_config_file.close();

    // 配置IK求解器
    robot_planning::IKSolverConfig ik_config;
    ik_config.urdf_path = urdf_path;
    ik_config.joint_limits_path = joint_limits_path;
    ik_config.init_config_path = init_config_path;

    robot_planning::IKSolver ik_solver(ik_config);

    // 从YAML文件读取目标位姿
    YAML::Node yaml_config;
    try
    {
        yaml_config = YAML::LoadFile(init_config_path);
    }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR("Failed to load YAML file: %s", e.what());
        res.success = false;
        res.message = "Failed to load YAML file: " + std::string(e.what());
        return true;
    }

    // 读取右臂目标位姿
    Eigen::Vector3d target_pos_R(yaml_config["target_pose_R"]["position"][0].as<double>(), yaml_config["target_pose_R"]["position"][1].as<double>(), yaml_config["target_pose_R"]["position"][2].as<double>());

    Eigen::Matrix3d target_ori_R;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            target_ori_R(i, j) = yaml_config["target_pose_R"]["orientation"][i][j].as<double>();
        }
    }

    // 读取左臂目标位姿
    Eigen::Vector3d target_pos_L(yaml_config["target_pose_L"]["position"][0].as<double>(), yaml_config["target_pose_L"]["position"][1].as<double>(), yaml_config["target_pose_L"]["position"][2].as<double>());

    Eigen::Matrix3d target_ori_L;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            target_ori_L(i, j) = yaml_config["target_pose_L"]["orientation"][i][j].as<double>();
        }
    }

    // 求解逆运动学
    auto result = ik_solver.solveIK(target_pos_R, target_ori_R, target_pos_L, target_ori_L);

    if (result.success)
    {
        // 保存结果到YAML文件
        std::string output_path = package_path + "/config/planned_trajectory.yaml";
        ik_solver.saveResult(output_path, result.joint_angles);
    }
    else
    {
        res.success = false;
        res.message = "IK solver failed: " + result.message;
        return true;
    }

    // 设置文件路径
    std::string init_floating_base_file = package_path + "/config/ik_urdf_double_arm_float.yaml";
    std::string gold_floating_base_file = package_path + "/config/planned_trajectory.yaml";
    std::string tf_using_file = package_path + "/config/tf_using.yaml";
    std::string output_file = package_path + "/config/leg_ik_cs.yaml";

    // 调用计算函数
    bool success = robot_planning::computeLegTransforms(init_floating_base_file, gold_floating_base_file, tf_using_file, output_file);

    if (!success)
    {
        res.success = false;
        res.message = "Failed to compute leg transforms!";
        ROS_ERROR("Failed to compute leg transforms!");
        return true;
    }

    // 调用腿的逆运动学求解
    auto leg_ik_result = robot_planning::solveLegIK(output_file, package_path + "/config/result_cs.yaml", "tf_mat_link1_0_flan1", "tf_mat_link4_0_flan4");

    if (leg_ik_result.success)
    {
        ROS_INFO("Leg IK computation completed successfully!");
    }
    else
    {
        res.success = false;
        res.message = "Failed to compute leg IK: " + leg_ik_result.message;
        ROS_ERROR("Failed to compute leg IK: %s", leg_ik_result.message.c_str());
        return true;
    }

    std::string result_cs_file = package_path + "/config/result_cs.yaml";

    // 整合4个分支与float_base的结果
    std::vector<std::vector<double>> joint_angles(TOTAL_POINT_NUM, std::vector<double>(24, 0.0));
    // 例如 1000 帧，每帧有 TOTAL_JOINT_NUM 个关节

    robot_planning::loadJointAngles(init_floating_base_file, gold_floating_base_file, result_cs_file, joint_angles);

    // 插值floating base
    std::vector<Eigen::Matrix4f> base_sequence = robot_planning::interpolateFloatingBase(init_floating_base_file, gold_floating_base_file, POINT_NUM);

    std::string planning_result_file = package_path + "/config/planning_result.yaml";
    // 将结果写出
    robot_planning::saveFullBodyTrajectoryToYAML(planning_result_file, joint_angles, base_sequence);

    // 设置响应
    res.success = true;
    res.message = "Path planning completed successfully!";
    ROS_INFO("Path planning completed successfully!");
    // 这里可以返回一些有用的信息，比如路径长度、规划时间等

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("plan_path", planCallback);
    ROS_INFO("Path planner node ready.");
    ros::spin();
    return 0;
}
