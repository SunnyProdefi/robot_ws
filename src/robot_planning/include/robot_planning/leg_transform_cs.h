#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace robot_planning
{

    /**
     * @brief 计算腿部变换矩阵
     * @param init_floating_base_file 初始浮动基座配置文件路径
     * @param gold_floating_base_file 目标浮动基座配置文件路径
     * @param tf_using_file 变换矩阵配置文件路径
     * @param output_file 输出文件路径
     * @return 是否成功执行
     */
    bool computeLegTransforms(const std::string& init_floating_base_file, const std::string& gold_floating_base_file, const std::string& tf_using_file, const std::string& output_file);

    /**
     * @brief 解析变换矩阵
     * @param node YAML节点
     * @return 4x4变换矩阵
     */
    Eigen::Matrix4f parseTransformMatrix(const YAML::Node& node);

    /**
     * @brief 保存变换矩阵到YAML文件
     * @param filename 输出文件路径
     * @param all_tf_link1_0_flan1 link1_0到flan1的变换矩阵列表
     * @param all_tf_link4_0_flan4 link4_0到flan4的变换矩阵列表
     */
    void saveToYAML(const std::string& filename, const std::vector<Eigen::Matrix4f>& all_tf_link1_0_flan1, const std::vector<Eigen::Matrix4f>& all_tf_link4_0_flan4);

}  // namespace robot_planning