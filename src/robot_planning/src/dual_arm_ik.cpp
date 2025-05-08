#include "robot_planning/dual_arm_ik.h"
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "robot_planning/ikfast_wrapper_single_arm.h"

namespace robot_planning
{
    std::vector<std::vector<float>> loadEEPosesFromYAML(const std::string& filename, const std::string& key)
    {
        YAML::Node yaml_data = YAML::LoadFile(filename);
        std::vector<std::vector<float>> poses;

        for (const auto& node : yaml_data)
        {
            if (!node[key])
            {
                std::cerr << "Missing key: " << key << std::endl;
                continue;
            }
            std::vector<float> pose = node[key].as<std::vector<float>>();
            if (pose.size() != 16)
            {
                std::cerr << "Invalid pose size for key: " << key << std::endl;
                continue;
            }
            poses.push_back(pose);
        }
        return poses;
    }

    void saveResultToYAML(const std::string& path, const std::vector<std::vector<std::vector<float>>>& all_left, const std::vector<std::vector<float>>& best_left, const std::vector<std::vector<std::vector<float>>>& all_right, const std::vector<std::vector<float>>& best_right)
    {
        YAML::Node root;

        YAML::Node all_left_node;
        for (const auto& group : all_left)
        {
            YAML::Node group_node;
            for (const auto& sol : group) group_node.push_back(sol);
            all_left_node.push_back(group_node);
        }
        root["all_left_solutions"] = all_left_node;

        YAML::Node best_left_node;
        for (const auto& sol : best_left) best_left_node.push_back(sol);
        root["best_left_solutions"] = best_left_node;

        YAML::Node all_right_node;
        for (const auto& group : all_right)
        {
            YAML::Node group_node;
            for (const auto& sol : group) group_node.push_back(sol);
            all_right_node.push_back(group_node);
        }
        root["all_right_solutions"] = all_right_node;

        YAML::Node best_right_node;
        for (const auto& sol : best_right) best_right_node.push_back(sol);
        root["best_right_solutions"] = best_right_node;

        std::ofstream fout(path);
        fout << root;
        fout.close();
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

    ArmIKResult solveArmIK(const std::string& yaml_file, const std::string& result_path, const std::string& tf_mat_link2_0_flan2, const std::string& tf_mat_link3_0_flan3)
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
        std::vector<float> q_l_init = {0.0, -1.2, 0.9, 1.5, -0.5, 0.0};
        std::vector<float> q_r_init = {0.0, -1.2, 0.9, -1.5, 0.5, 0.0};

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
