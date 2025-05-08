#ifndef ROBOT_PLANNING_DUAL_ARM_IK_H
#define ROBOT_PLANNING_DUAL_ARM_IK_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "robot_planning/ikfast_wrapper_single_arm.h"

#pragma once

#include <vector>
#include <string>

namespace robot_planning {
// IK 计算结果结构体
struct ArmIKResult
{
    bool success;
    std::string message;

    std::vector<std::vector<std::vector<float>>> all_left_solutions;
    std::vector<std::vector<float>> best_left_solutions;

    std::vector<std::vector<std::vector<float>>> all_right_solutions;
    std::vector<std::vector<float>> best_right_solutions;
};

// 加载期望末端姿态列表（world → flan）
// 每条记录为 4x4 行优先列表（16 floats）
std::vector<std::vector<float>> loadEEPosesFromYAML(const std::string& yaml_file, const std::string& key);

// 保存逆解结果到 YAML
void saveResultToYAML(const std::string& path,
                      const std::vector<std::vector<std::vector<float>>>& all_left,
                      const std::vector<std::vector<float>>& best_left,
                      const std::vector<std::vector<std::vector<float>>>& all_right,
                      const std::vector<std::vector<float>>& best_right);

// 逆运动学主函数
ArmIKResult solveArmIK(const std::string& yaml_file,
                       const std::string& result_path,
                       const std::string& tf_mat_link2_0_flan2,
                       const std::string& tf_mat_link3_0_flan3);

} // namespace robot_planning

#endif  // ROBOT_PLANNING_DUAL_ARM_IK_H