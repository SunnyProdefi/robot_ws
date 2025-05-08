#include "robot_planning/dual_arm_ik.h"
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "robot_planning/ikfast_wrapper_single_arm.h"

namespace robot_planning
{
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

    void saveResultToYAML(const std::string& filename, const std::vector<std::vector<std::vector<float>>>& all_left_solutions, const std::vector<std::vector<float>>& best_left_solutions, const std::vector<std::vector<std::vector<float>>>& all_right_solutions,
                          const std::vector<std::vector<float>>& best_right_solutions)
    {
        YAML::Node result;

        // === 左臂所有解 ===
        YAML::Node left_arm_solutions;
        for (const auto& solution_group : all_left_solutions)
        {
            YAML::Node group_node;
            for (const auto& sol : solution_group)
            {
                YAML::Node joint_set;
                for (float val : sol) joint_set.push_back(val);
                joint_set.SetStyle(YAML::EmitterStyle::Flow);
                group_node.push_back(joint_set);
            }
            left_arm_solutions.push_back(group_node);
        }
        result["left_arm_solutions"] = left_arm_solutions;

        // === 左臂最优解 ===
        YAML::Node best_left_arm_solutions;
        for (const auto& sol : best_left_solutions)
        {
            YAML::Node joint_set;
            for (float val : sol) joint_set.push_back(val);
            joint_set.SetStyle(YAML::EmitterStyle::Flow);
            best_left_arm_solutions.push_back(joint_set);
        }
        result["best_left_arm_solutions"] = best_left_arm_solutions;

        // === 右臂所有解 ===
        YAML::Node right_arm_solutions;
        for (const auto& solution_group : all_right_solutions)
        {
            YAML::Node group_node;
            for (const auto& sol : solution_group)
            {
                YAML::Node joint_set;
                for (float val : sol) joint_set.push_back(val);
                joint_set.SetStyle(YAML::EmitterStyle::Flow);
                group_node.push_back(joint_set);
            }
            right_arm_solutions.push_back(group_node);
        }
        result["right_arm_solutions"] = right_arm_solutions;

        // === 右臂最优解 ===
        YAML::Node best_right_arm_solutions;
        for (const auto& sol : best_right_solutions)
        {
            YAML::Node joint_set;
            for (float val : sol) joint_set.push_back(val);
            joint_set.SetStyle(YAML::EmitterStyle::Flow);
            best_right_arm_solutions.push_back(joint_set);
        }
        result["best_right_arm_solutions"] = best_right_arm_solutions;

        // === 写入 YAML 文件 ===
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

    float calculateDistance(const std::vector<float>& a, const std::vector<float>& b)
    {
        float d = 0;
        for (size_t i = 0; i < a.size(); ++i)
        {
            float diff = a[i] - b[i];
            d += diff * diff;
        }
        return std::sqrt(d);
    }

    float normalizeAngleToNearest(float angle, float ref)
    {
        const float PI = M_PI, TWO_PI = 2 * M_PI;
        float diff = angle - ref;
        while (diff > PI) diff -= TWO_PI;
        while (diff < -PI) diff += TWO_PI;
        return ref + diff;
    }

    std::pair<std::vector<std::vector<float>>, std::vector<float>> findAllSolutions(const std::vector<float>& ik_result, const std::vector<float>& q_ref, size_t dof)
    {
        std::vector<std::vector<float>> all;
        std::vector<float> best;
        float min_dist = std::numeric_limits<float>::max();

        for (size_t i = 0; i + dof <= ik_result.size(); i += dof)
        {
            std::vector<float> q(ik_result.begin() + i, ik_result.begin() + i + dof);
            for (int idx : {3, 5}) q[idx] = normalizeAngleToNearest(q[idx], q_ref[idx]);
            all.push_back(q);
            float d = calculateDistance(q, q_ref);
            if (d < min_dist)
            {
                min_dist = d;
                best = q;
            }
        }
        return {all, best};
    }

    ArmIKResult solveArmIK(const std::string& yaml_file, const std::string& result_path, const std::string& tf_mat_link2_0_flan2, const std::string& tf_mat_link3_0_flan3, const std::vector<double>& branch2_init, const std::vector<double>& branch3_init)
    {
        ArmIKResult result;
        result.success = false;

        std::vector<std::vector<float>> ee_poses_l = loadEEPosesFromYAML(yaml_file, tf_mat_link2_0_flan2);
        std::vector<std::vector<float>> ee_poses_r = loadEEPosesFromYAML(yaml_file, tf_mat_link3_0_flan3);

        if (ee_poses_l.empty() || ee_poses_r.empty())
        {
            result.message = "Failed to load EE poses.";
            return result;
        }

        robots::Kinematics kin;

        std::vector<float> q_l_init(branch2_init.begin(), branch2_init.end());
        std::vector<float> q_r_init(branch3_init.begin(), branch3_init.end());

        for (size_t i = 0; i < ee_poses_l.size(); ++i)
        {
            auto q_l_raw = kin.inverse(ee_poses_l[i]);
            auto q_r_raw = kin.inverse(ee_poses_r[i]);

            auto [q_l_all, q_l_best] = findAllSolutions(q_l_raw, q_l_init, kin.num_of_joints);
            auto [q_r_all, q_r_best] = findAllSolutions(q_r_raw, q_r_init, kin.num_of_joints);

            result.all_left_solutions.push_back(q_l_all);
            result.best_left_solutions.push_back(q_l_best);
            result.all_right_solutions.push_back(q_r_all);
            result.best_right_solutions.push_back(q_r_best);

            q_l_init = q_l_best;
            q_r_init = q_r_best;
        }

        saveResultToYAML(result_path, result.all_left_solutions, result.best_left_solutions, result.all_right_solutions, result.best_right_solutions);

        result.success = true;
        result.message = "Arm IK computation completed successfully.";
        return result;
    }
}
