#include <iostream>
#include <vector>
#include <cmath>            // 误差计算
#include <limits>           // 用于 std::numeric_limits
#include <yaml-cpp/yaml.h>  // 读取 YAML 文件
#include <fstream>          // 用于写入 YAML 文件
#include "robot_planning/leg_ik_cs.h"
#include "robot_planning/ikfast_wrapper_single_arm.h"

namespace robot_planning
{
    using namespace robots;  // 使用 robots 命名空间

    // **从 YAML 读取多个 ee_pose（转换为 4x4 矩阵格式）**
    std::vector<std::vector<float>> loadEEPosesFromYAML(const std::string& filename, const std::string& tf_name)
    {
        YAML::Node yaml_data;
        try
        {
            yaml_data = YAML::LoadFile(filename);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error loading YAML file: " << e.what() << std::endl;
            return {};
        }

        std::vector<std::vector<float>> all_poses;

        // 逐步解析 YAML 文件中的步骤
        for (const auto& step : yaml_data)
        {
            // 确保目标矩阵（tf_name）存在于当前步骤
            if (step.second[tf_name])
            {
                YAML::Node pose_data = step.second[tf_name]["target_pose"];

                // 解析 position 和 orientation
                std::vector<float> position;
                for (int i = 0; i < 3; ++i)
                {
                    position.push_back(pose_data["position"][i].as<float>());
                }

                // 解析旋转矩阵（orientation）
                std::vector<float> rotation_matrix;
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        rotation_matrix.push_back(pose_data["orientation"][i][j].as<float>());
                    }
                }

                // 组合 position 和 rotation_matrix 成 3x4 矩阵
                std::vector<float> ee_pose;
                for (int row = 0; row < 3; ++row)
                {
                    for (int col = 0; col < 3; ++col)
                    {
                        ee_pose.push_back(rotation_matrix[row * 3 + col]);
                    }
                    ee_pose.push_back(position[row]);
                }

                all_poses.push_back(ee_pose);
            }
            else
            {
                std::cerr << "Error: " << tf_name << " not found in step " << step.first.as<std::string>() << std::endl;
            }
        }

        return all_poses;
    }

    // 将角度归一化到 [0, 2π)
    float normalizeAngle(float angle)
    {
        const float TWO_PI = 2.0f * static_cast<float>(M_PI);
        return fmod(angle + TWO_PI, TWO_PI);
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

    // **保存计算结果到 YAML**
    void saveResultToYAML(const std::string& filename, const std::vector<std::vector<std::vector<float>>>& all_left_solutions, const std::vector<std::vector<float>>& best_left_solutions, const std::vector<std::vector<std::vector<float>>>& all_right_solutions,
                          const std::vector<std::vector<float>>& best_right_solutions)
    {
        YAML::Node result;

        // **存储左腿所有解**
        YAML::Node left_leg_solutions;
        for (const auto& solutions : all_left_solutions)
        {
            YAML::Node solution_set;
            for (const auto& sol : solutions)
            {
                YAML::Node joint_set;
                for (float value : sol)
                {
                    joint_set.push_back(value);
                }
                joint_set.SetStyle(YAML::EmitterStyle::Flow);
                solution_set.push_back(joint_set);
            }
            left_leg_solutions.push_back(solution_set);
        }
        result["left_leg_solutions"] = left_leg_solutions;

        // **存储最优解**
        YAML::Node best_left_leg_solutions;
        for (const auto& best_solution : best_left_solutions)
        {
            YAML::Node joint_set;
            for (float value : best_solution)
            {
                joint_set.push_back(value);
            }
            joint_set.SetStyle(YAML::EmitterStyle::Flow);
            best_left_leg_solutions.push_back(joint_set);
        }
        result["best_left_leg_solutions"] = best_left_leg_solutions;

        // **存储右腿所有解**
        YAML::Node right_leg_solutions;
        for (const auto& solutions : all_right_solutions)
        {
            YAML::Node solution_set;
            for (const auto& sol : solutions)
            {
                YAML::Node joint_set;
                for (float value : sol)
                {
                    joint_set.push_back(value);
                }
                joint_set.SetStyle(YAML::EmitterStyle::Flow);
                solution_set.push_back(joint_set);
            }
            right_leg_solutions.push_back(solution_set);
        }
        result["right_leg_solutions"] = right_leg_solutions;

        // **存储最优解**
        YAML::Node best_right_leg_solutions;
        for (const auto& best_solution : best_right_solutions)
        {
            YAML::Node joint_set;
            for (float value : best_solution)
            {
                joint_set.push_back(value);
            }
            joint_set.SetStyle(YAML::EmitterStyle::Flow);
            best_right_leg_solutions.push_back(joint_set);
        }
        result["best_right_leg_solutions"] = best_right_leg_solutions;

        // **写入 YAML 文件**
        std::ofstream fout(filename);
        if (fout.is_open())
        {
            fout << result;
            fout.close();
            std::cout << "IK results saved to " << filename << std::endl;
        }
        else
        {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
        }
    }

    LegIKResult solveLegIK(const std::string& yaml_file, const std::string& result_path, const std::string& tf_mat_link1_0_flan1, const std::string& tf_mat_link4_0_flan4)
    {
        LegIKResult result;
        result.success = false;

        // **从 YAML 读取多个 ee_pose**
        std::vector<std::vector<float>> ee_poses_l = loadEEPosesFromYAML(yaml_file, tf_mat_link1_0_flan1);
        std::vector<std::vector<float>> ee_poses_r = loadEEPosesFromYAML(yaml_file, tf_mat_link4_0_flan4);

        if (ee_poses_l.empty() || ee_poses_r.empty())
        {
            result.message = "Failed to load ee_pose from YAML file.";
            return result;
        }

        // **创建 Kinematics 对象**
        robots::Kinematics kinematics;

        // **初始化关节角**
        std::vector<float> leg_l_q_init = {1.597727, 0.295055, 2.156446, 3.101495, -0.494894, -0.016370};
        std::vector<float> leg_r_q_init = {-1.597727, 0.295055, 2.156446, -0.040097, 0.494959, 0.016244};

        std::vector<std::vector<std::vector<float>>> all_left_solutions;
        std::vector<std::vector<float>> best_left_solutions;

        std::vector<std::vector<std::vector<float>>> all_right_solutions;
        std::vector<std::vector<float>> best_right_solutions;

        // **遍历所有目标姿态并计算解**
        for (size_t i = 0; i < ee_poses_l.size(); ++i)
        {
            // **计算逆运动学解**
            std::vector<float> leg_l_q_res = kinematics.inverse(ee_poses_l[i]);
            std::vector<float> leg_r_q_res = kinematics.inverse(ee_poses_r[i]);

            // **计算所有解并找到最优解**
            auto [left_solutions, best_left] = findAllSolutions(leg_l_q_res, leg_l_q_init, kinematics.num_of_joints);
            auto [right_solutions, best_right] = findAllSolutions(leg_r_q_res, leg_r_q_init, kinematics.num_of_joints);

            all_left_solutions.push_back(left_solutions);
            best_left_solutions.push_back(best_left);

            all_right_solutions.push_back(right_solutions);
            best_right_solutions.push_back(best_right);

            leg_l_q_init = best_left;
            leg_r_q_init = best_right;
        }

        // **保存结果**
        saveResultToYAML(result_path, all_left_solutions, best_left_solutions, all_right_solutions, best_right_solutions);

        result.all_left_solutions = all_left_solutions;
        result.best_left_solutions = best_left_solutions;
        result.all_right_solutions = all_right_solutions;
        result.best_right_solutions = best_right_solutions;
        result.success = true;
        result.message = "Leg IK computation completed successfully";

        return result;
    }

}  // namespace robot_planning
