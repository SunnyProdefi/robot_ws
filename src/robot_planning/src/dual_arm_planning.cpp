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
            throw std::runtime_error("Transform name '" + transform_name + "' not found in YAML file");

        YAML::Node target_pose = config[transform_name]["target_pose"];
        if (!target_pose)
            throw std::runtime_error("Missing 'target_pose' under '" + transform_name + "'");

        // 读取 position
        YAML::Node position = target_pose["position"];
        if (!position || position.size() != 3)
            throw std::runtime_error("Position must be a list of 3 elements under '" + transform_name + "'");

        transform(0, 3) = position[0].as<double>();
        transform(1, 3) = position[1].as<double>();
        transform(2, 3) = position[2].as<double>();

        // 读取 orientation（3x3）
        YAML::Node orientation = target_pose["orientation"];
        if (!orientation || orientation.size() != 3)
            throw std::runtime_error("Orientation must be a 3x3 matrix under '" + transform_name + "'");

        for (int i = 0; i < 3; ++i)
        {
            YAML::Node row = orientation[i];
            if (!row || row.size() != 3)
                throw std::runtime_error("Orientation row " + std::to_string(i) + " must have 3 elements");

            for (int j = 0; j < 3; ++j)
            {
                try
                {
                    transform(i, j) = row[j].as<double>();
                }
                catch (const YAML::TypedBadConversion<double>& e)
                {
                    std::ostringstream oss;
                    oss << "Bad conversion at orientation[" << i << "][" << j << "] = '" << row[j] << "'";
                    throw std::runtime_error(oss.str());
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[loadTransformFromYAML] Error parsing '" << transform_name << "' in " << file_path << ": " << e.what());
        throw;  // 向上传递错误
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
void saveInterpolatedPosesToYAML(const std::string& path, const std::vector<Eigen::Matrix4d>& poses, const Eigen::Matrix4d& tf_mat_cube_m_l, const Eigen::Matrix4d& tf_mat_cube_m_r,
                                 const Eigen::Matrix4d& tf_mat_world_base_link,  // ← from req.float_base_pose
                                 const Eigen::Matrix4d& tf_mat_base_link2_0,     // ← from tf_using.yaml
                                 const Eigen::Matrix4d& tf_mat_base_link3_0)     // ← from tf_using.yaml
{
    YAML::Node root;

    for (const auto& pose : poses)
    {
        // 世界坐标系下的 flan2 / flan3
        Eigen::Matrix4d tf_world_flan2 = pose * tf_mat_cube_m_l;
        Eigen::Matrix4d tf_world_flan3 = pose * tf_mat_cube_m_r;

        // 链式反向变换：linkX_0 → flanX
        Eigen::Matrix4d tf_link2_0_flan2 = tf_mat_base_link2_0.inverse() * tf_mat_world_base_link.inverse() * tf_world_flan2;

        Eigen::Matrix4d tf_link3_0_flan3 = tf_mat_base_link3_0.inverse() * tf_mat_world_base_link.inverse() * tf_world_flan3;

        // 写入 YAML，格式为嵌套矩阵
        YAML::Node node;
        YAML::Node left_matrix, right_matrix;
        for (int i = 0; i < 4; ++i)
        {
            std::vector<float> row_l, row_r;
            for (int j = 0; j < 4; ++j)
            {
                row_l.push_back(tf_link2_0_flan2(i, j));
                row_r.push_back(tf_link3_0_flan3(i, j));
            }
            left_matrix.push_back(row_l);
            right_matrix.push_back(row_r);
        }

        node["tf_mat_link2_0_flan2"] = left_matrix;
        node["tf_mat_link3_0_flan3"] = right_matrix;
        root.push_back(node);
    }

    std::ofstream fout(path);
    if (!fout.is_open())
        throw std::runtime_error("Failed to open file: " + path);
    fout << root;
    fout.close();
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

    ArmIKResult ik_result = solveArmIK(pose_yaml_path, result_yaml_path, tf_link2_0_flan2, tf_link3_0_flan3);

    //

    // === 5. 返回结果 ===
    res.success = ik_result.success;
    res.message = ik_result.message;

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
