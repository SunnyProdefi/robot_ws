#pragma once

#include <vector>
#include <Eigen/Dense>
#include <string>

namespace robot_planning
{
    // 用枚举来给 41 个关节分配索引 ID，方便快速访问
    enum JointID : int
    {
        // Branch 1
        JOINT1_1,
        JOINT1_2,
        JOINT1_3,
        JOINT1_4,
        JOINT1_5,
        JOINT1_6,

        // Branch 2
        JOINT2_1,
        JOINT2_2,
        JOINT2_3,
        JOINT2_4,
        JOINT2_5,
        JOINT2_6,

        // Branch 3
        JOINT3_1,
        JOINT3_2,
        JOINT3_3,
        JOINT3_4,
        JOINT3_5,
        JOINT3_6,

        // Branch 4
        JOINT4_1,
        JOINT4_2,
        JOINT4_3,
        JOINT4_4,
        JOINT4_5,
        JOINT4_6,

        // 用来标记总数量，必须放在最后
        JOINT_COUNT
    };

    // 建立一个数组，把上面枚举的每个 ID 与具体关节名称一一对应
    static const std::array<std::string, JOINT_COUNT> JOINT_NAMES = {

        /* JOINT1_1 */ "Joint1_1",
        /* JOINT1_2 */ "Joint1_2",
        /* JOINT1_3 */ "Joint1_3",
        /* JOINT1_4 */ "Joint1_4",
        /* JOINT1_5 */ "Joint1_5",
        /* JOINT1_6 */ "Joint1_6",

        /* JOINT2_1 */ "Joint2_1",
        /* JOINT2_2 */ "Joint2_2",
        /* JOINT2_3 */ "Joint2_3",
        /* JOINT2_4 */ "Joint2_4",
        /* JOINT2_5 */ "Joint2_5",
        /* JOINT2_6 */ "Joint2_6",

        /* JOINT3_1 */ "Joint3_1",
        /* JOINT3_2 */ "Joint3_2",
        /* JOINT3_3 */ "Joint3_3",
        /* JOINT3_4 */ "Joint3_4",
        /* JOINT3_5 */ "Joint3_5",
        /* JOINT3_6 */ "Joint3_6",

        /* JOINT4_1 */ "Joint4_1",
        /* JOINT4_2 */ "Joint4_2",
        /* JOINT4_3 */ "Joint4_3",
        /* JOINT4_4 */ "Joint4_4",
        /* JOINT4_5 */ "Joint4_5",
        /* JOINT4_6 */ "Joint4_6",
    };

    /**
     * @brief 加载并插值关节角度
     * @param joint_angle_file   存放 init_joint_angles / gold_joint_angles 的 YAML 文件（如 joint_angle.yaml）
     * @param result_cs_file     存放 best_left_leg_solutions / best_right_leg_solutions 的 YAML 文件（如 result_cs.yaml）
     * @param joint_angles       输出的插值后关节角度序列（外部需先给定其 size）
     */
    void loadJointAngles(const std::string& init_floating_base_file, const std::string& gold_floating_base_file, const std::string& result_cs_file, std::vector<std::vector<double>>& joint_angles);

    /**
     * @brief 插值 floating_base（从 floating_base.yaml）
     * @param floating_base_file   文件名
     * @param point_per_stage      每个阶段的插值步长
     * @return 插值后的位姿序列
     */
    std::vector<Eigen::Matrix4f> interpolateFloatingBase(const std::string& init_floating_base_file, const std::string& gold_floating_base_file, int point_per_stage = 250);
    std::vector<Eigen::Matrix4f> interpolateFloatingBase_home(const std::string& init_floating_base_file, const std::string& gold_floating_base_file, int point_per_stage = 250);

    /**
     * @brief 将 joint + base 结果写入 YAML 文件
     * @param output_file               输出文件名
     * @param joint_angles              最终插值后的关节角度序列
     * @param floating_base_sequence    最终插值后的基座序列
     * @return 写入是否成功
     */
    bool saveFullBodyTrajectoryToYAML(const std::string& output_file, const std::vector<std::vector<double>>& joint_angles, const std::vector<Eigen::Matrix4f>& floating_base_sequence);

}  // namespace robot_planning
