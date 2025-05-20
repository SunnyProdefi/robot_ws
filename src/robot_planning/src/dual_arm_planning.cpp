#include <ros/ros.h>
#include "robot_planning/PlanDualArmPath.h"
#include "robot_planning/dual_arm_ik.h"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>
#include <stdexcept>
#include <sstream>

using namespace robot_planning;

// 从YAML文件加载变换矩阵
Eigen::Matrix4d loadTransformFromYAML(const std::string& file_path, const std::string& transform_name)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    try
    {
        YAML::Node config = YAML::LoadFile(file_path);
        if (!config[transform_name])
            throw std::runtime_error("Missing transform: " + transform_name);

        YAML::Node target_pose = config[transform_name]["target_pose"];
        if (!target_pose)
            throw std::runtime_error("Missing target_pose for: " + transform_name);

        YAML::Node position = target_pose["position"];
        if (!position || position.size() != 3)
            throw std::runtime_error("Position invalid size in: " + transform_name);

        for (int i = 0; i < 3; ++i)
        {
            double val = position[i].as<double>();
            // ROS_INFO_STREAM(transform_name << ".position[" << i << "] = " << val);
            transform(i, 3) = val;
        }

        YAML::Node orientation = target_pose["orientation"];
        if (!orientation || orientation.size() != 3)
            throw std::runtime_error("Orientation invalid size in: " + transform_name);

        for (int i = 0; i < 3; ++i)
        {
            YAML::Node row = orientation[i];
            if (!row || row.size() != 3)
                throw std::runtime_error("Orientation[" + std::to_string(i) + "] invalid in: " + transform_name);

            for (int j = 0; j < 3; ++j)
            {
                try
                {
                    double val = row[j].as<double>();
                    // ROS_INFO_STREAM(transform_name << ".orientation[" << i << "][" << j << "] = " << val);
                    transform(i, j) = val;
                }
                catch (const YAML::TypedBadConversion<double>& e)
                {
                    std::ostringstream oss;
                    oss << "Bad conversion at orientation[" << i << "][" << j << "]: raw=" << row[j];
                    throw std::runtime_error(oss.str());
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[loadTransformFromYAML] Failed for " << transform_name << ": " << e.what());
        throw;
    }

    return transform;
}

std::vector<Eigen::Matrix4d> interpolateSE3(const Eigen::Matrix4d& T_start, const Eigen::Matrix4d& T_end, int num_points)
{
    std::vector<Eigen::Matrix4d> results;

    Eigen::Vector3d p_start = T_start.block<3, 1>(0, 3);
    Eigen::Vector3d p_end = T_end.block<3, 1>(0, 3);

    Eigen::Matrix3d R_start = T_start.block<3, 3>(0, 0);
    Eigen::Matrix3d R_end = T_end.block<3, 3>(0, 0);

    Eigen::Quaterniond q_start(R_start);
    Eigen::Quaterniond q_end(R_end);

    for (int i = 1; i <= num_points; ++i)
    {
        float t = static_cast<float>(i) / (num_points + 1);

        Eigen::Vector3d p_interp = (1 - t) * p_start + t * p_end;
        Eigen::Quaterniond q_interp = q_start.slerp(t, q_end);

        Eigen::Matrix4d T_interp = Eigen::Matrix4d::Identity();
        T_interp.block<3, 3>(0, 0) = q_interp.toRotationMatrix();
        T_interp.block<3, 1>(0, 3) = p_interp;

        results.push_back(T_interp);
    }

    return results;
}

Eigen::Matrix4d parseMatrix(const geometry_msgs::Transform& tf_msg)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    // 平移
    mat(0, 3) = tf_msg.translation.x;
    mat(1, 3) = tf_msg.translation.y;
    mat(2, 3) = tf_msg.translation.z;
    // 旋转（四元数转旋转矩阵）
    Eigen::Quaterniond q(tf_msg.rotation.w, tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z);
    mat.block<3, 3>(0, 0) = q.toRotationMatrix();
    return mat;
}

// 工具函数：保存插值点到 YAML
void saveInterpolatedPosesToYAML(const std::string& path, const std::vector<Eigen::Matrix4d>& poses, const Eigen::Matrix4d& tf_mat_cube_m_l, const Eigen::Matrix4d& tf_mat_cube_m_r, const Eigen::Matrix4d& tf_mat_world_base_link, const Eigen::Matrix4d& tf_mat_base_link2_0,
                                 const Eigen::Matrix4d& tf_mat_base_link3_0)
{
    YAML::Node root;

    for (size_t i = 0; i < poses.size(); ++i)
    {
        const auto& pose = poses[i];

        Eigen::Matrix4d tf_world_flan2 = pose * tf_mat_cube_m_l;
        Eigen::Matrix4d tf_world_flan3 = pose * tf_mat_cube_m_r;

        Eigen::Matrix4d tf_link2_0_flan2 = tf_mat_base_link2_0.inverse() * tf_mat_world_base_link.inverse() * tf_world_flan2;
        Eigen::Matrix4d tf_link3_0_flan3 = tf_mat_base_link3_0.inverse() * tf_mat_world_base_link.inverse() * tf_world_flan3;

        // 构建 step_i 节点
        std::string step_key = "step_" + std::to_string(i);
        YAML::Node step_node;

        auto buildTargetPose = [](const Eigen::Matrix4d& tf) -> YAML::Node
        {
            YAML::Node target_pose;
            // position 一行表示
            YAML::Node pos;
            pos.SetStyle(YAML::EmitterStyle::Flow);
            pos.push_back(tf(0, 3));
            pos.push_back(tf(1, 3));
            pos.push_back(tf(2, 3));
            target_pose["position"] = pos;

            // orientation 三行嵌套，每行一行表示
            YAML::Node ori;
            for (int r = 0; r < 3; ++r)
            {
                YAML::Node row;
                row.SetStyle(YAML::EmitterStyle::Flow);
                for (int c = 0; c < 3; ++c)
                {
                    row.push_back(tf(r, c));
                }
                ori.push_back(row);
            }
            target_pose["orientation"] = ori;
            return target_pose;
        };

        step_node["tf_mat_link2_0_flan2"]["target_pose"] = buildTargetPose(tf_link2_0_flan2);
        step_node["tf_mat_link3_0_flan3"]["target_pose"] = buildTargetPose(tf_link3_0_flan3);

        root[step_key] = step_node;
    }

    std::ofstream fout(path);
    if (!fout.is_open())
        throw std::runtime_error("Failed to open file for writing: " + path);

    fout << root;
    fout.close();
    ROS_INFO_STREAM("Successfully wrote interpolated poses to " << path);
}

bool planDualArmPath(robot_planning::PlanDualArmPath::Request& req, robot_planning::PlanDualArmPath::Response& res)
{
    // === 1. 读取初始与目标位姿 ===
    Eigen::Matrix4d tf_mat_world_cube_m_init = parseMatrix(req.tf_mat_world_cube_m_init);
    Eigen::Matrix4d tf_mat_world_cube_m_goal = parseMatrix(req.tf_mat_world_cube_m_goal);

    Eigen::Matrix4d tf_mat_cube_m_r = parseMatrix(req.tf_mat_cube_m_r);  // 右手相对
    Eigen::Matrix4d tf_mat_cube_m_l = parseMatrix(req.tf_mat_cube_m_l);  // 左手相对

    // === 2. 插值路径点 ===
    std::vector<Eigen::Matrix4d> interpolated_cube_poses = interpolateSE3(tf_mat_world_cube_m_init, tf_mat_world_cube_m_goal, req.num_interpolations);

    // === 3. 保存路径点到 YAML（用于 IK）===
    std::string package_path = ros::package::getPath("robot_planning");
    std::string pose_yaml_path = package_path + "/config/dual_arm_interpolated_poses.yaml";
    std::string tf_using_path = package_path + "/config/tf_using.yaml";
    // base_link->world的坐标系从请求获取req.float_base_pose
    // tf_mat_base_link2_0和tf_mat_base_link3_0是从tf_using.yaml中获取的
    // saveInterpolatedPosesToYAML中储存的是tf_mat_link2_0_flan2和tf_mat_link3_0_flan3
    if (req.float_base_pose.size() != 7)
    {
        res.success = false;
        res.message = "float_base_pose must have 7 elements: [x, y, z, qx, qy, qz, qw]";
        return true;
    }

    Eigen::Vector3d t(req.float_base_pose[0], req.float_base_pose[1], req.float_base_pose[2]);
    Eigen::Quaterniond q(req.float_base_pose[6],  // qw
                         req.float_base_pose[3],  // qx
                         req.float_base_pose[4],  // qy
                         req.float_base_pose[5]   // qz
    );
    q.normalize();  // 安全起见

    Eigen::Matrix4d tf_mat_world_base_link = Eigen::Matrix4d::Identity();
    tf_mat_world_base_link.block<3, 3>(0, 0) = q.toRotationMatrix();
    tf_mat_world_base_link.block<3, 1>(0, 3) = t;
    Eigen::Matrix4d tf_mat_base_link2_0, tf_mat_base_link3_0;
    try
    {
        tf_mat_base_link2_0 = loadTransformFromYAML(tf_using_path, "tf_mat_base_link2_0");
        tf_mat_base_link3_0 = loadTransformFromYAML(tf_using_path, "tf_mat_base_link3_0");
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Failed to parse tf_using.yaml: " << e.what());
        res.success = false;
        res.message = e.what();
        return true;
    }

    saveInterpolatedPosesToYAML(pose_yaml_path, interpolated_cube_poses, tf_mat_cube_m_l, tf_mat_cube_m_r, tf_mat_world_base_link, tf_mat_base_link2_0, tf_mat_base_link3_0);

    // === 4. 执行逆运动学求解（对应 flan2 和 flan3）===
    std::string result_yaml_path = package_path + "/config/dual_arm_ik_result.yaml";  // TODO: 自定义路径
    std::string tf_link2_0_flan2 = "tf_mat_link2_0_flan2";
    std::string tf_link3_0_flan3 = "tf_mat_link3_0_flan3";

    // 从请求中获取分支2和3的初始关节角度
    const std::vector<double> q_init2 = req.branch2_joints;
    const std::vector<double> q_init3 = req.branch3_joints;
    ArmIKResult ik_result = solveArmIK(pose_yaml_path, result_yaml_path, tf_link2_0_flan2, tf_link3_0_flan3, q_init2, q_init3);

    // 从 YAML 文件加载 IK 结果 并保存到响应float64[] joint_trajectory 中

    // === 读取最优解轨迹并拼接为 joint_trajectory ===
    try
    {
        YAML::Node result = YAML::LoadFile(result_yaml_path);
        const auto& best_left = result["best_left_arm_solutions"];
        const auto& best_right = result["best_right_arm_solutions"];

        if (!best_left || !best_right || best_left.size() != best_right.size())
        {
            ROS_ERROR("Mismatch in left/right best solution sizes or missing fields.");
            res.success = false;
            res.message = "Invalid or mismatched IK solution size.";
            return true;
        }

        for (size_t i = 0; i < best_left.size(); ++i)
        {
            const YAML::Node& q_left = best_left[i];
            const YAML::Node& q_right = best_right[i];

            if (q_left.size() != 6 || q_right.size() != 6)
            {
                ROS_WARN_STREAM("One of the joint vectors at step " << i << " is not size 6.");
                continue;
            }

            for (size_t j = 0; j < 6; ++j) res.joint_trajectory.push_back(q_left[j].as<double>());

            for (size_t j = 0; j < 6; ++j) res.joint_trajectory.push_back(q_right[j].as<double>());
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Failed to parse IK result YAML: " << e.what());
        res.success = false;
        res.message = "Failed to parse IK result YAML: " + std::string(e.what());
        return true;
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_arm_planner");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("plan_dual_arm_path", planDualArmPath);
    ROS_INFO("Dual arm planning service is ready.");
    ros::spin();
    return 0;
}
