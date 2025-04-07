#include <ros/ros.h>
#include "robot_planning/PlanPath.h"
#include "robot_planning/ik_solver.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <Eigen/Dense>

bool planCallback(robot_planning::PlanPath::Request& req, robot_planning::PlanPath::Response& res)
{
    ROS_INFO("Received planning request...");

    // 配置IK求解器
    robot_planning::IKSolverConfig config;
    config.urdf_path = "../models/double_arms_description.urdf";
    config.joint_limits_path = "../config/joint_limits.yaml";
    config.init_config_path = "../config/ik_urdf_double_arm_float.yaml";
    
    robot_planning::IKSolver ik_solver(config);

    // 从YAML文件读取目标位姿
    YAML::Node config = YAML::LoadFile("../config/ik_urdf_double_arm_float.yaml");

    // 读取右臂目标位姿
    Eigen::Vector3d target_pos_R(
        config["target_pose_R"]["position"][0].as<double>(),
        config["target_pose_R"]["position"][1].as<double>(),
        config["target_pose_R"]["position"][2].as<double>()
    );

    Eigen::Matrix3d target_ori_R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            target_ori_R(i, j) = config["target_pose_R"]["orientation"][i][j].as<double>();
        }
    }

    // 读取左臂目标位姿
    Eigen::Vector3d target_pos_L(
        config["target_pose_L"]["position"][0].as<double>(),
        config["target_pose_L"]["position"][1].as<double>(),
        config["target_pose_L"]["position"][2].as<double>()
    );

    Eigen::Matrix3d target_ori_L;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            target_ori_L(i, j) = config["target_pose_L"]["orientation"][i][j].as<double>();
        }
    }

    // 求解逆运动学
    auto result = ik_solver.solveIK(target_pos_R, target_ori_R, target_pos_L, target_ori_L);

    if (result.success) {
        // 保存结果到YAML文件
        std::string output_path = "/home/prodefi/planned_trajectory.yaml";
        ik_solver.saveResult(output_path, result.joint_angles);
        
        res.success = true;
        res.message = "Trajectory saved to " + output_path;
    } else {
        res.success = false;
        res.message = "IK solver failed: " + result.message;
    }

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
