#include <ros/ros.h>
#include <robot_planning/CartesianInterpolation.h>
#include <robot_planning/ikfast_wrapper_single_arm.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace Eigen;

class CartesianInterpolationServer
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    robots::Kinematics kinematics_;

    // 计算所有解，并找到最优解（并对第4个关节归一化后替换原数据）
    std::pair<std::vector<std::vector<float>>, std::vector<float>> findAllSolutions(std::vector<float>& ik_results,       // 输入所有 IK 解的一维向量
                                                                                    const std::vector<float>& initial_q,  // 初始关节角
                                                                                    size_t num_joints                     // 关节数量
    )
    {
        std::vector<std::vector<float>> all_solutions;
        std::vector<float> closest_solution;
        float min_distance = std::numeric_limits<float>::max();

        // 定义关节限位
        std::vector<std::pair<float, float>> joint_limits = {
            {-2.3562, 4.7124},  // Joint2_1
            {-2.0944, 1.5708},  // Joint2_2
            {-2.5307, 2.5307},  // Joint2_3
            {-3.1416, 3.1416},  // Joint2_4
            {-1.9199, 1.9199},  // Joint2_5
            {-3.1416, 3.1416}   // Joint2_6
        };

        for (size_t i = 0; i < ik_results.size(); i += num_joints)
        {
            std::vector<float> solution(ik_results.begin() + i, ik_results.begin() + i + num_joints);

            // 检查是否在限位内
            bool within_limits = true;
            for (size_t j = 0; j < solution.size(); ++j)
            {
                if (solution[j] < joint_limits[j].first || solution[j] > joint_limits[j].second)
                {
                    within_limits = false;
                    break;
                }
            }

            if (within_limits)
            {
                // ✅ 对周期关节做归一化处理
                std::vector<size_t> normalize_indices = {3, 5};
                for (size_t joint_index : normalize_indices)
                {
                    if (joint_index < solution.size() && joint_index < initial_q.size())
                    {
                        solution[joint_index] = normalizeAngleToNearest(solution[joint_index], initial_q[joint_index]);
                    }
                }

                all_solutions.push_back(solution);

                float distance = calculateDistance(initial_q, solution);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    closest_solution = solution;
                }
            }
        }

        return {all_solutions, closest_solution};
    }

    // 欧几里得距离计算
    float calculateDistance(const std::vector<float>& a, const std::vector<float>& b)
    {
        float distance = 0.0f;
        for (size_t i = 0; i < a.size(); ++i)
        {
            float diff = a[i] - b[i];
            distance += diff * diff;
        }
        return std::sqrt(distance);
    }

    // 归一化 angle 到以 reference 为中心的最近等效角度
    float normalizeAngleToNearest(float angle, float reference)
    {
        const float TWO_PI = 2.0f * static_cast<float>(M_PI);
        float diff = angle - reference;

        while (diff > M_PI) diff -= TWO_PI;
        while (diff < -M_PI) diff += TWO_PI;

        return reference + diff;
    }

    // 四元数球面线性插值
    Vector4d slerp(const Vector4d& q1, const Vector4d& q2, double t)
    {
        // 计算点积
        double dot = q1.dot(q2);

        // 如果点积为负，反转一个四元数以取最短路径
        Vector4d q2_ = q2;
        if (dot < 0.0)
        {
            q2_ = -q2_;
            dot = -dot;
        }

        // 如果四元数非常接近，使用线性插值
        if (dot > 0.9995)
        {
            return (q1 + t * (q2_ - q1)).normalized();
        }

        // 执行球面线性插值
        double theta_0 = std::acos(dot);
        double theta = theta_0 * t;
        double sin_theta = std::sin(theta);
        double sin_theta_0 = std::sin(theta_0);

        double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        double s1 = sin_theta / sin_theta_0;

        return (s0 * q1 + s1 * q2_).normalized();
    }

    // 笛卡尔空间插值
    bool interpolateCartesian(const std::vector<double>& start_pose, const std::vector<double>& goal_pose, double duration, double frequency, std::vector<std::vector<double>>& cartesian_path)
    {
        if (start_pose.size() != 7 || goal_pose.size() != 7)
        {
            ROS_ERROR("Start and goal poses must be 7-dimensional (position + quaternion)");
            return false;
        }

        if (duration <= 0.0 || frequency <= 0.0)
        {
            ROS_ERROR("Duration and frequency must be positive");
            return false;
        }

        // 计算插值点数
        int num_points = static_cast<int>(duration * frequency) + 1;
        if (num_points < 2)
        {
            ROS_ERROR("Number of points must be at least 2");
            return false;
        }

        // 提取位置和四元数
        Vector3d start_pos(start_pose[0], start_pose[1], start_pose[2]);
        Vector3d goal_pos(goal_pose[0], goal_pose[1], goal_pose[2]);
        Vector4d start_quat(start_pose[6], start_pose[3], start_pose[4], start_pose[5]);
        Vector4d goal_quat(goal_pose[6], goal_pose[3], goal_pose[4], goal_pose[5]);

        // 归一化四元数
        start_quat.normalize();
        goal_quat.normalize();

        cartesian_path.clear();
        cartesian_path.reserve(num_points);

        for (int i = 0; i < num_points; ++i)
        {
            double t = static_cast<double>(i) / (num_points - 1);

            // 位置线性插值
            Vector3d pos = start_pos + t * (goal_pos - start_pos);

            // 四元数球面线性插值
            Vector4d quat = slerp(start_quat, goal_quat, t);

            // 组合成完整的位姿
            std::vector<double> pose = {pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]};

            // ROSINFO打印
            // ROS_INFO("Interpolated pose %d: [%f, %f, %f, %f, %f, %f, %f]", i, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);

            cartesian_path.push_back(pose);
        }

        return true;
    }

    bool handleInterpolationRequest(robot_planning::CartesianInterpolation::Request& req, robot_planning::CartesianInterpolation::Response& res)
    {
        ROS_INFO("Starting Cartesian interpolation request...");

        // 打印req
        ROS_INFO("Branch ID: %d", req.branch_id);
        ROS_INFO("Start pose: [%f, %f, %f, %f, %f, %f, %f]", req.start_pose[0], req.start_pose[1], req.start_pose[2], req.start_pose[3], req.start_pose[4], req.start_pose[5], req.start_pose[6]);
        ROS_INFO("Goal pose: [%f, %f, %f, %f, %f, %f, %f]", req.goal_pose[0], req.goal_pose[1], req.goal_pose[2], req.goal_pose[3], req.goal_pose[4], req.goal_pose[5], req.goal_pose[6]);
        ROS_INFO("Joint angles: [%f, %f, %f, %f, %f, %f]", req.joint_angles[0], req.joint_angles[1], req.joint_angles[2], req.joint_angles[3], req.joint_angles[4], req.joint_angles[5]);
        ROS_INFO("Duration: %f", req.duration);
        ROS_INFO("Frequency: %f", req.frequency);

        // 执行笛卡尔空间插值
        std::vector<std::vector<double>> cartesian_path;
        if (!interpolateCartesian(req.start_pose, req.goal_pose, req.duration, req.frequency, cartesian_path))
        {
            res.success = false;
            res.message = "Failed to interpolate Cartesian path";
            ROS_ERROR("Failed to interpolate Cartesian path");
            return true;
        }

        ROS_INFO("Successfully interpolated Cartesian path with %lu waypoints.", cartesian_path.size());

        // 初始关节角
        std::vector<float> q_init(req.joint_angles.begin(), req.joint_angles.end());
        ROS_INFO("Initial joint angles set: [%f, %f, %f, %f, %f, %f]", q_init[0], q_init[1], q_init[2], q_init[3], q_init[4], q_init[5]);

        // 存储最终的关节轨迹
        std::vector<float> joint_trajectory;

        // 创建YAML文档
        YAML::Node yaml_doc;
        yaml_doc["waypoints"] = YAML::Node(YAML::NodeType::Sequence);  // 显式初始化为序列

        for (size_t i = 0; i < cartesian_path.size(); ++i)
        {
            const auto& pose = cartesian_path[i];

            // Convert 7-element pose (position + quaternion) to 12-element transformation matrix
            Eigen::Vector3d pos(pose[0], pose[1], pose[2]);
            Eigen::Quaterniond q(pose[3], pose[4], pose[5], pose[6]);  // (w, x, y, z)
            q.normalize();

            // Create 3x4 transformation matrix
            std::vector<float> ee_pose(12);
            Eigen::Matrix3d rot = q.toRotationMatrix();

            // Fill rotation matrix
            ee_pose[0] = rot(0, 0);
            ee_pose[1] = rot(0, 1);
            ee_pose[2] = rot(0, 2);
            ee_pose[3] = pos[0];
            ee_pose[4] = rot(1, 0);
            ee_pose[5] = rot(1, 1);
            ee_pose[6] = rot(1, 2);
            ee_pose[7] = pos[1];
            ee_pose[8] = rot(2, 0);
            ee_pose[9] = rot(2, 1);
            ee_pose[10] = rot(2, 2);
            ee_pose[11] = pos[2];

            // 得到全部 IK 解（扁平向量）
            std::vector<float> ik_results = kinematics_.inverse(ee_pose);

            if (ik_results.empty())
            {
                res.success = false;
                res.message = "Failed to find IK solution for intermediate pose";
                ROS_ERROR("No IK solution found for intermediate pose at waypoint %lu", i);
                return true;
            }

            // 使用 findAllSolutions 寻找最优解
            auto [all_solutions, best_solution] = findAllSolutions(ik_results, q_init, kinematics_.num_of_joints);

            if (best_solution.empty())
            {
                res.success = false;
                res.message = "No valid IK solution after filtering";
                ROS_ERROR("No valid IK solution after filtering for waypoint %lu", i);
                return true;
            }

            // TODO:
            // 添加每帧的执行时间戳；
            // 改为速度规划（而不是固定频率）；
            // 做角速度 / 加速度限制（更高阶插值如 B样条）；

            if (i > 0)
            {
                std::vector<float> prev_solution = q_init;

                size_t dof = prev_solution.size();
                float max_diff = 0.0f;

                for (size_t j = 0; j < dof; ++j)
                {
                    float diff = std::abs(best_solution[j] - prev_solution[j]);
                    max_diff = std::max(max_diff, diff);
                }

                float max_time_required = max_diff / 1.0f;                          // max_joint_velocity
                int steps = static_cast<int>(std::ceil(max_time_required / 0.02));  // time_step

                for (int k = 1; k <= steps; ++k)
                {
                    float t = static_cast<float>(k) / (steps + 1);
                    std::vector<float> interpolated(dof);

                    for (size_t j = 0; j < dof; ++j)
                    {
                        interpolated[j] = prev_solution[j] + t * (best_solution[j] - prev_solution[j]);
                    }

                    joint_trajectory.insert(joint_trajectory.end(), interpolated.begin(), interpolated.end());

                    // 记录插值点到YAML
                    YAML::Node interp_node;
                    interp_node["waypoint_index"] = static_cast<double>(i) + k * 0.01;
                    interp_node["type"] = "interpolated";
                    interp_node["interpolated_from"].push_back(i - 1);
                    interp_node["interpolated_from"].push_back(i);

                    YAML::Node angle_node;
                    for (float val : interpolated)
                    {
                        angle_node.push_back(val);
                    }
                    interp_node["joint_angles"] = angle_node;

                    yaml_doc["waypoints"].push_back(interp_node);
                }
            }

            // 插入当前最优解
            joint_trajectory.insert(joint_trajectory.end(), best_solution.begin(), best_solution.end());
            q_init = best_solution;

            // ==========================
            // 保存当前最优解到 YAML
            // ==========================
            YAML::Node waypoint_node;
            waypoint_node["waypoint_index"] = static_cast<int>(i);
            waypoint_node["type"] = "original";

            // 添加当前最优解角度
            YAML::Node best_solution_node;
            for (size_t j = 0; j < best_solution.size(); ++j)
            {
                best_solution_node.push_back(best_solution[j]);
            }
            waypoint_node["joint_angles"] = best_solution_node;
            waypoint_node["best_solution"] = best_solution_node;

            // 添加所有解（可选）
            YAML::Node all_solutions_node;
            for (const auto& solution : all_solutions)
            {
                YAML::Node solution_node;
                for (size_t j = 0; j < solution.size(); ++j)
                {
                    solution_node.push_back(solution[j]);
                }
                all_solutions_node.push_back(solution_node);
            }
            waypoint_node["all_solutions"] = all_solutions_node;

            // 添加到主 YAML 结构
            yaml_doc["waypoints"].push_back(waypoint_node);
        }

        // ==========================
        // 保存 YAML 文件
        // ==========================
        std::string save_path = "/home/prodefi/github/robot_ws/src/robot_planning/config/ik_solutions.yaml";
        YAML::Emitter out;
        out.SetIndent(2);
        out.SetMapFormat(YAML::Block);
        out.SetSeqFormat(YAML::Block);  // 默认 block，但我们单独设置数组为 flow
        out << YAML::BeginMap;
        out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;

        for (const auto& wp : yaml_doc["waypoints"])
        {
            out << YAML::BeginMap;

            for (const auto& keyval : wp)
            {
                std::string key = keyval.first.as<std::string>();
                out << YAML::Key << key;

                // 单独处理数组，设置为 Flow
                if (key == "joint_angles" || key == "best_solution" || key == "interpolated_from")
                {
                    out << YAML::Value << YAML::Flow << keyval.second;
                }
                else if (key == "all_solutions")
                {
                    out << YAML::Value << YAML::BeginSeq;
                    for (const auto& sol : keyval.second)
                    {
                        out << YAML::Flow << sol;
                    }
                    out << YAML::EndSeq;
                }
                else
                {
                    out << YAML::Value << keyval.second;
                }
            }

            out << YAML::EndMap;
        }

        out << YAML::EndSeq;
        out << YAML::EndMap;

        std::ofstream fout(save_path);
        fout << out.c_str();
        fout.close();

        ROS_INFO_STREAM("IK solutions saved to: " << save_path);

        // ==========================
        // 设置 ROS 服务响应
        // ==========================
        res.joint_trajectory.assign(joint_trajectory.begin(), joint_trajectory.end());
        res.success = true;
        res.message = "Successfully generated joint trajectory with optimal IK solutions";

        ROS_INFO("Successfully generated joint trajectory with %lu joint positions.", joint_trajectory.size());
        return true;
    }

public:
    CartesianInterpolationServer() { service_ = nh_.advertiseService("cartesian_interpolation", &CartesianInterpolationServer::handleInterpolationRequest, this); }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cartesian_interpolation_server");
    CartesianInterpolationServer server;
    ROS_INFO("Cartesian interpolation server is ready");
    ros::spin();
    return 0;
}