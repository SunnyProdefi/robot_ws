#include "robot_planning/full_body_trajectory.h"
#include "robot_planning/yaml_common.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace robot_planning
{

    /**
     * @brief 加载并插值关节角度
     * @param joint_angle_file   存放 init_joint_angles / gold_joint_angles 的 YAML 文件
     * @param result_cs_file     存放 best_left_leg_solutions / best_right_leg_solutions 的 YAML 文件
     * @param joint_angles       输出的插值后关节角度序列（外部需先给定其 size，比如 N=1000）
     */
    void loadJointAngles(const std::string& init_floating_base_file, const std::string& gold_floating_base_file, const std::string& result_cs_file, std::vector<std::vector<double>>& joint_angles)
    {
        // ============= 第一部分：从 joint_angle.yaml 里加载并插值 JOINT2_* 和 JOINT3_* =============
        YAML::Node init_config = loadYAML(init_floating_base_file);
        YAML::Node gold_config = loadYAML(gold_floating_base_file);
        if (!init_config || !gold_config)
        {
            ROS_ERROR("YAML file for init_floating_base_file or gold_floating_base_file cannot be loaded.");
            return;
        }

        if (init_config["init_joint_angles"] && gold_config["gold_joint_angles"])
        {
            std::vector<double> init_joint_angles = init_config["init_joint_angles"].as<std::vector<double>>();
            std::vector<double> gold_joint_angles = gold_config["gold_joint_angles"].as<std::vector<double>>();

            // 映射到 JOINT2_* 和 JOINT3_* 对应索引（仅插值这 12 个关节）
            std::array<int, 12> joint_indices = {JOINT2_1, JOINT2_2, JOINT2_3, JOINT2_4, JOINT2_5, JOINT2_6, JOINT3_1, JOINT3_2, JOINT3_3, JOINT3_4, JOINT3_5, JOINT3_6};

            int steps = static_cast<int>(joint_angles.size());  // 例如 1000

            if (steps < 2)
            {
                ROS_ERROR("joint_angles size is too small for interpolation.");
                return;
            }

            for (size_t i = 0; i < joint_indices.size(); ++i)
            {
                int joint_id = joint_indices[i];
                double init_value = init_joint_angles[i];
                double goal_value = gold_joint_angles[i];

                // 插值填充每一帧的角度值
                for (int step = 0; step < steps; ++step)
                {
                    double t = static_cast<double>(step) / (steps - 1);  // [0, 1]
                    double interpolated_value = init_value + t * (goal_value - init_value);
                    joint_angles[step][joint_id] = interpolated_value;
                }
            }
        }
        else
        {
            ROS_ERROR("joint_angle.yaml missing init_joint_angles or gold_joint_angles.");
        }

        // ============= 第二部分：从 result_cs.yaml 里读取 JOINT1_* 和 JOINT4_* =============
        YAML::Node result = loadYAML(result_cs_file);
        if (!result)
        {
            ROS_ERROR("YAML file for result_cs_file cannot be loaded.");
            return;
        }

        if (!result["best_left_leg_solutions"] || !result["best_right_leg_solutions"])
        {
            ROS_ERROR("Missing best_left_leg_solutions or best_right_leg_solutions in result_cs.yaml");
            return;
        }

        const YAML::Node& left_solutions = result["best_left_leg_solutions"];
        const YAML::Node& right_solutions = result["best_right_leg_solutions"];

        // 与插值后关节角度的帧数对比
        if (left_solutions.size() != joint_angles.size() || right_solutions.size() != joint_angles.size())
        {
            ROS_ERROR("Mismatch in joint angle size and solution steps.");
            return;
        }

        // JOINT1_* 对应的索引
        std::array<int, 6> joint1_indices = {JOINT1_1, JOINT1_2, JOINT1_3, JOINT1_4, JOINT1_5, JOINT1_6};
        // JOINT4_* 对应的索引
        std::array<int, 6> joint4_indices = {JOINT4_1, JOINT4_2, JOINT4_3, JOINT4_4, JOINT4_5, JOINT4_6};

        for (size_t step = 0; step < joint_angles.size(); ++step)
        {
            // best_left_leg_solutions[step] 是一个长度6的向量
            std::vector<double> left_angles = left_solutions[step].as<std::vector<double>>();
            std::vector<double> right_angles = right_solutions[step].as<std::vector<double>>();

            if (left_angles.size() != 6 || right_angles.size() != 6)
            {
                ROS_ERROR_STREAM("Invalid angle vector size at step " << step);
                continue;
            }

            // 写入 JOINT1_* 角度
            for (size_t i = 0; i < 6; ++i)
            {
                joint_angles[step][joint1_indices[i]] = left_angles[i];
                joint_angles[step][joint4_indices[i]] = right_angles[i];
            }
        }

        ROS_INFO("Loaded JOINT1_* and JOINT4_* from result_cs.yaml");
    }

    /**
     * @brief 插值 floating_base
     * @param floating_base_file 文件名，如 floating_base.yaml
     * @param point_per_stage    每个阶段的插值点数，默认 250
     * @return 逐阶段插值后得到的基座变换序列
     */
    std::vector<Eigen::Matrix4f> interpolateFloatingBase(const std::string& init_floating_base_file, const std::string& gold_floating_base_file, int point_per_stage)
    {
        YAML::Node init_floating_base_yaml = loadYAML(init_floating_base_file);
        YAML::Node gold_floating_base_yaml = loadYAML(gold_floating_base_file);
        std::vector<Eigen::Matrix4f> interpolated_poses;

        if (!init_floating_base_yaml || !gold_floating_base_yaml)
        {
            ROS_ERROR("Failed to load floating_base.yaml.");
            return interpolated_poses;
        }

        // 解析初始位姿
        Eigen::Vector3f pos_init(init_floating_base_yaml["init_floating_base"][0].as<float>(), init_floating_base_yaml["init_floating_base"][1].as<float>(), init_floating_base_yaml["init_floating_base"][2].as<float>());
        Eigen::Matrix3f rot_init =
            quaternionToRotationMatrix(init_floating_base_yaml["init_floating_base"][3].as<float>(), init_floating_base_yaml["init_floating_base"][4].as<float>(), init_floating_base_yaml["init_floating_base"][5].as<float>(), init_floating_base_yaml["init_floating_base"][6].as<float>());

        // 解析目标位姿
        Eigen::Vector3f pos_goal(gold_floating_base_yaml["gold_floating_base"][0].as<float>(), gold_floating_base_yaml["gold_floating_base"][1].as<float>(), gold_floating_base_yaml["gold_floating_base"][2].as<float>());
        Eigen::Matrix3f rot_goal =
            quaternionToRotationMatrix(gold_floating_base_yaml["gold_floating_base"][3].as<float>(), gold_floating_base_yaml["gold_floating_base"][4].as<float>(), gold_floating_base_yaml["gold_floating_base"][5].as<float>(), gold_floating_base_yaml["gold_floating_base"][6].as<float>());

        // 四元数用于插值旋转
        Eigen::Quaternionf q_init(rot_init);
        Eigen::Quaternionf q_goal(rot_goal);

        // 初始化插值时的“当前”位置和姿态
        Eigen::Vector3f pos_current = pos_init;
        Eigen::Quaternionf q_current = q_init;

        /**
         *  分阶段插值:
         *  stage0: x方向
         *  stage1: y方向
         *  stage2: 旋转
         *  stage3: z方向
         */
        for (int stage = 0; stage < 4; ++stage)
        {
            for (int i = 0; i < point_per_stage; ++i)
            {
                float t = (point_per_stage == 1) ? 1.0f : static_cast<float>(i) / (point_per_stage - 1);

                if (stage == 0)
                {
                    // 插值X
                    pos_current.x() = (1 - t) * pos_init.x() + t * pos_goal.x();
                }
                else if (stage == 1)
                {
                    // 插值Y
                    pos_current.y() = (1 - t) * pos_init.y() + t * pos_goal.y();
                }
                else if (stage == 2)
                {
                    // 插值旋转 (四元数slerp)
                    q_current = q_init.slerp(t, q_goal);
                }
                else if (stage == 3)
                {
                    // 插值Z
                    pos_current.z() = (1 - t) * pos_init.z() + t * pos_goal.z();
                }

                // 组装矩阵
                Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
                tf.block<3, 3>(0, 0) = q_current.toRotationMatrix();
                tf.block<3, 1>(0, 3) = pos_current;
                interpolated_poses.push_back(tf);
            }
        }

        return interpolated_poses;
    }

    /**
     * @brief 插值 floating_base
     * @param floating_base_file 文件名，如 floating_base.yaml
     * @param point_per_stage    每个阶段的插值点数，默认 250
     * @return 逐阶段插值后得到的基座变换序列
     */
    std::vector<Eigen::Matrix4f> interpolateFloatingBase_home(const std::string& init_floating_base_file, const std::string& gold_floating_base_file, int point_per_stage)
    {
        YAML::Node init_floating_base_yaml = loadYAML(init_floating_base_file);
        YAML::Node gold_floating_base_yaml = loadYAML(gold_floating_base_file);
        std::vector<Eigen::Matrix4f> interpolated_poses;

        if (!init_floating_base_yaml || !gold_floating_base_yaml)
        {
            ROS_ERROR("Failed to load floating_base.yaml.");
            return interpolated_poses;
        }

        // 解析初始位姿
        Eigen::Vector3f pos_init(init_floating_base_yaml["init_floating_base"][0].as<float>(), init_floating_base_yaml["init_floating_base"][1].as<float>(), init_floating_base_yaml["init_floating_base"][2].as<float>());
        Eigen::Matrix3f rot_init =
            quaternionToRotationMatrix(init_floating_base_yaml["init_floating_base"][3].as<float>(), init_floating_base_yaml["init_floating_base"][4].as<float>(), init_floating_base_yaml["init_floating_base"][5].as<float>(), init_floating_base_yaml["init_floating_base"][6].as<float>());

        // 解析目标位姿
        Eigen::Vector3f pos_goal(gold_floating_base_yaml["gold_floating_base"][0].as<float>(), gold_floating_base_yaml["gold_floating_base"][1].as<float>(), gold_floating_base_yaml["gold_floating_base"][2].as<float>());
        Eigen::Matrix3f rot_goal =
            quaternionToRotationMatrix(gold_floating_base_yaml["gold_floating_base"][3].as<float>(), gold_floating_base_yaml["gold_floating_base"][4].as<float>(), gold_floating_base_yaml["gold_floating_base"][5].as<float>(), gold_floating_base_yaml["gold_floating_base"][6].as<float>());

        Eigen::Quaternionf q_init(rot_init);
        Eigen::Quaternionf q_goal(rot_goal);

        Eigen::Vector3f pos_current = pos_init;
        Eigen::Quaternionf q_current = q_init;

        /**
         *  修改后的阶段顺序:
         *  stage0: Z
         *  stage1: 旋转
         *  stage2: Y
         *  stage3: X
         */
        for (int stage = 0; stage < 4; ++stage)
        {
            for (int i = 0; i < point_per_stage; ++i)
            {
                float t = (point_per_stage == 1) ? 1.0f : static_cast<float>(i) / (point_per_stage - 1);

                if (stage == 0)
                {
                    // 插值 Z
                    pos_current.z() = (1 - t) * pos_init.z() + t * pos_goal.z();
                }
                else if (stage == 1)
                {
                    // 插值旋转 (四元数 slerp)
                    q_current = q_init.slerp(t, q_goal);
                }
                else if (stage == 2)
                {
                    // 插值 Y
                    pos_current.y() = (1 - t) * pos_init.y() + t * pos_goal.y();
                }
                else if (stage == 3)
                {
                    // 插值 X
                    pos_current.x() = (1 - t) * pos_init.x() + t * pos_goal.x();
                }

                // 构造变换矩阵
                Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
                tf.block<3, 3>(0, 0) = q_current.toRotationMatrix();
                tf.block<3, 1>(0, 3) = pos_current;
                interpolated_poses.push_back(tf);
            }
        }

        return interpolated_poses;
    }

    /**
     * @brief 将 joint + base 结果写入 YAML 文件
     * @param output_file 输出文件名
     * @param joint_angles 最终插值后的关节角度序列
     * @param floating_base_sequence 最终插值后的基座序列
     * @return 是否保存成功
     */
    bool saveFullBodyTrajectoryToYAML(const std::string& output_file, const std::vector<std::vector<double>>& joint_angles, const std::vector<Eigen::Matrix4f>& floating_base_sequence)
    {
        if (joint_angles.size() != floating_base_sequence.size())
        {
            ROS_ERROR("Mismatched size between joint angles and base pose sequence");
            return false;
        }

        // 先创建一个 root 节点
        YAML::Node root;

        // 准备两个顶层 Sequence，分别存放 joint 和 base
        // （可以显式设成 Block Style，这样会更直观一些）
        YAML::Node joint_angle_seq(YAML::NodeType::Sequence);
        joint_angle_seq.SetStyle(YAML::EmitterStyle::Block);

        YAML::Node floating_base_seq(YAML::NodeType::Sequence);
        floating_base_seq.SetStyle(YAML::EmitterStyle::Block);

        // 遍历每一帧
        for (size_t i = 0; i < joint_angles.size(); ++i)
        {
            // ------------ 1) 写入本帧的关节角度 ------------
            // 用一个 Flow Style 的 Sequence 来存放关节角度，这样每个子序列就是一行
            YAML::Node anglesNode(YAML::NodeType::Sequence);
            anglesNode.SetStyle(YAML::EmitterStyle::Flow);

            for (double val : joint_angles[i])
            {
                anglesNode.push_back(val);
            }

            // 将子序列 push 到外层的 joint_angle_seq
            joint_angle_seq.push_back(anglesNode);

            // ------------ 2) 写入本帧的浮动基座 ------------
            Eigen::Matrix4f tf = floating_base_sequence[i];
            Eigen::Matrix3f R = tf.block<3, 3>(0, 0);
            Eigen::Vector3f T = tf.block<3, 1>(0, 3);
            Eigen::Quaternionf q(R);

            // 同理，也用一个 Flow Style 的 Sequence
            YAML::Node baseNode(YAML::NodeType::Sequence);
            baseNode.SetStyle(YAML::EmitterStyle::Flow);
            baseNode.push_back(T.x());
            baseNode.push_back(T.y());
            baseNode.push_back(T.z());
            baseNode.push_back(q.x());
            baseNode.push_back(q.y());
            baseNode.push_back(q.z());
            baseNode.push_back(q.w());

            floating_base_seq.push_back(baseNode);
        }

        // 把这两个顶层节点放入 root
        root["joint_angle_sequence"] = joint_angle_seq;
        root["floating_base_sequence"] = floating_base_seq;

        // 写入文件
        std::ofstream fout(output_file);
        if (!fout.is_open())
        {
            ROS_ERROR_STREAM("Cannot open file to write: " << output_file);
            return false;
        }

        fout << root;
        fout.close();

        ROS_INFO_STREAM("Saved full body trajectory to YAML: " << output_file);
        return true;
    }

}  // namespace robot_planning
