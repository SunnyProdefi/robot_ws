#include <ros/ros.h>
#include <robot_planning/CartesianInterpolation.h>
#include <robot_planning/ikfast_wrapper_single_arm.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

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

        for (size_t i = 0; i < ik_results.size(); i += num_joints)
        {
            // 索引安全检查
            if (i + num_joints > ik_results.size())
                break;

            // 获取第4个关节角度（index = 3）
            size_t joint4_index = i + 3;
            if (joint4_index < ik_results.size())
            {
                float joint4_angle = ik_results[joint4_index];

                // ✅ 判断是否接近 ±π，若是则归一化
                if (std::abs(std::abs(joint4_angle) - static_cast<float>(M_PI)) < 1.0f)
                {
                    float normalized = normalizeAngle(joint4_angle);
                    ik_results[joint4_index] = normalized;
                }
            }

            // 构造当前解
            std::vector<float> solution(ik_results.begin() + i, ik_results.begin() + i + num_joints);
            all_solutions.push_back(solution);

            // 计算距离并判断是否为最优解
            float distance = calculateDistance(initial_q, solution);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_solution = solution;
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

    // 将角度归一化到 [0, 2π)
    float normalizeAngle(float angle)
    {
        const float TWO_PI = 2.0f * static_cast<float>(M_PI);
        return fmod(angle + TWO_PI, TWO_PI);
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
        Vector4d start_quat(start_pose[3], start_pose[4], start_pose[5], start_pose[6]);
        Vector4d goal_quat(goal_pose[3], goal_pose[4], goal_pose[5], goal_pose[6]);

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

            cartesian_path.push_back(pose);
        }

        return true;
    }

    bool handleInterpolationRequest(robot_planning::CartesianInterpolation::Request& req, robot_planning::CartesianInterpolation::Response& res)
    {
        // 执行笛卡尔空间插值
        std::vector<std::vector<double>> cartesian_path;
        if (!interpolateCartesian(req.start_pose, req.goal_pose, req.duration, req.frequency, cartesian_path))
        {
            res.success = false;
            res.message = "Failed to interpolate Cartesian path";
            return true;
        }

        // 初始关节角
        std::vector<float> q_init(req.joint_angles.begin(), req.joint_angles.end());

        // 存储最终的关节轨迹
        std::vector<float> joint_trajectory;

        for (const auto& pose : cartesian_path)
        {
            std::vector<float> ee_pose(pose.begin(), pose.end());

            // 得到全部 IK 解（扁平向量）
            std::vector<float> ik_results = kinematics_.inverse(ee_pose);

            if (ik_results.empty())
            {
                res.success = false;
                res.message = "Failed to find IK solution for intermediate pose";
                return true;
            }

            // 使用 findAllSolutions 寻找最优解
            auto [all_solutions, best_solution] = findAllSolutions(ik_results, q_init, kinematics_.num_of_joints);

            if (best_solution.empty())
            {
                res.success = false;
                res.message = "No valid IK solution after filtering";
                return true;
            }

            // 添加最优解到轨迹中
            joint_trajectory.insert(joint_trajectory.end(), best_solution.begin(), best_solution.end());

            // 更新 q_init 为当前最优解，用于下一步的最优性参考
            q_init = best_solution;
        }

        // 设置响应
        res.joint_trajectory.assign(joint_trajectory.begin(), joint_trajectory.end());
        res.success = true;
        res.message = "Successfully generated joint trajectory with optimal IK solutions";

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