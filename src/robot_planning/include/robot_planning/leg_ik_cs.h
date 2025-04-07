#ifndef ROBOT_PLANNING_LEG_IK_CS_H
#define ROBOT_PLANNING_LEG_IK_CS_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "robot_planning/ikfast_wrapper_single_arm.h"

namespace robot_planning {

struct LegIKResult {
    std::vector<std::vector<std::vector<float>>> all_left_solutions;
    std::vector<std::vector<float>> best_left_solutions;
    std::vector<std::vector<std::vector<float>>> all_right_solutions;
    std::vector<std::vector<float>> best_right_solutions;
    bool success;
    std::string message;
};

// Function to solve leg IK for both legs
LegIKResult solveLegIK(const std::string& yaml_file, 
                       const std::string& result_path,
                       const std::string& tf_mat_link1_0_flan1,
                       const std::string& tf_mat_link4_0_flan4);

} // namespace robot_planning

#endif // ROBOT_PLANNING_LEG_IK_CS_H 