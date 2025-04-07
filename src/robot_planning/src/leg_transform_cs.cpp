#include "robot_planning/leg_transform_cs.h"
#include <fstream>
#include <ros/package.h>

namespace robot_planning {

YAML::Node loadYAML(const std::string& filename)
{
    try
    {
        return YAML::LoadFile(filename);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Failed to load YAML file: " << filename << ". Error: " << e.what());
        return YAML::Node();
    }
}

Eigen::Matrix3f quaternionToRotationMatrix(float x, float y, float z, float w)
{
    Eigen::Quaternionf q(w, x, y, z);
    return q.toRotationMatrix();
}

Eigen::Matrix4f parseTransformMatrix(const YAML::Node& node)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    if (!node)
    {
        ROS_ERROR("Invalid transform node.");
        return transform;
    }
    // 读取 position
    Eigen::Vector3f position(node["target_pose"]["position"][0].as<float>(), node["target_pose"]["position"][1].as<float>(), node["target_pose"]["position"][2].as<float>());

    // 读取 orientation
    Eigen::Matrix3f rotation;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rotation(i, j) = node["target_pose"]["orientation"][i][j].as<float>();
        }
    }

    // 组合到 4x4 变换矩阵
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = position;
    return transform;
}

void saveToYAML(const std::string& filename, const std::vector<Eigen::Matrix4f>& all_tf_link1_0_flan1, const std::vector<Eigen::Matrix4f>& all_tf_link4_0_flan4)
{
    YAML::Node root;

    // 遍历每个步骤，将数据存储在 YAML 结构中
    for (int step = 0; step < all_tf_link1_0_flan1.size(); ++step)
    {
        YAML::Node tf_data;

        // 处理 tf_mat_link1_0_flan1
        Eigen::Matrix4f tf = all_tf_link1_0_flan1[step];
        YAML::Node position;
        position.push_back(tf(0, 3));
        position.push_back(tf(1, 3));
        position.push_back(tf(2, 3));
        position.SetStyle(YAML::EmitterStyle::Flow);
        tf_data["position"] = position;

        // 旋转矩阵
        YAML::Node orientation;
        for (int r = 0; r < 3; ++r)
        {
            YAML::Node row;
            for (int c = 0; c < 3; ++c)
            {
                row.push_back(tf(r, c));
            }
            row.SetStyle(YAML::EmitterStyle::Flow);
            orientation.push_back(row);
        }
        tf_data["orientation"] = orientation;

        root["step_" + std::to_string(step)]["tf_mat_link1_0_flan1"]["target_pose"] = tf_data;

        // 处理 tf_mat_link4_0_flan4
        tf = all_tf_link4_0_flan4[step];
        tf_data = YAML::Node();  // 使用新的空节点替代清空操作

        position = YAML::Node();  // 重新创建空节点
        position.push_back(tf(0, 3));
        position.push_back(tf(1, 3));
        position.push_back(tf(2, 3));
        position.SetStyle(YAML::EmitterStyle::Flow);
        tf_data["position"] = position;

        orientation = YAML::Node();  // 重新创建空节点
        for (int r = 0; r < 3; ++r)
        {
            YAML::Node row;
            for (int c = 0; c < 3; ++c)
            {
                row.push_back(tf(r, c));
            }
            row.SetStyle(YAML::EmitterStyle::Flow);
            orientation.push_back(row);
        }
        tf_data["orientation"] = orientation;

        root["step_" + std::to_string(step)]["tf_mat_link4_0_flan4"]["target_pose"] = tf_data;
    }

    // 一次性写入 YAML 文件
    std::ofstream file_out(filename);
    if (file_out.is_open())
    {
        file_out << root;  // 将数据写入文件
        file_out.close();
        ROS_INFO_STREAM("Saved all transformations to " << filename);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to open file " << filename);
    }
}

bool computeLegTransforms(
    const std::string& init_floating_base_file,
    const std::string& gold_floating_base_file,
    const std::string& tf_using_file,
    const std::string& output_file)
{
    YAML::Node init_floating_base_yaml = loadYAML(init_floating_base_file);
    YAML::Node gold_floating_base_yaml = loadYAML(gold_floating_base_file);
    YAML::Node tf_using_yaml = loadYAML(tf_using_file);

    if (!init_floating_base_yaml || !gold_floating_base_yaml || !tf_using_yaml)
    {
        ROS_ERROR("Failed to load one or both YAML files.");
        return false;
    }

    // **读取 init_floating_base（world -> base）**
    Eigen::Vector3f pos_world_base_init(init_floating_base_yaml["init_floating_base"][0].as<float>(), init_floating_base_yaml["init_floating_base"][1].as<float>(), init_floating_base_yaml["init_floating_base"][2].as<float>());
    Eigen::Matrix3f rot_world_base_init =
        quaternionToRotationMatrix(init_floating_base_yaml["init_floating_base"][3].as<float>(), init_floating_base_yaml["init_floating_base"][4].as<float>(), init_floating_base_yaml["init_floating_base"][5].as<float>(), init_floating_base_yaml["init_floating_base"][6].as<float>());
    Eigen::Matrix4f init_tf_mat_world_base = Eigen::Matrix4f::Identity();
    init_tf_mat_world_base.block<3, 3>(0, 0) = rot_world_base_init;
    init_tf_mat_world_base.block<3, 1>(0, 3) = pos_world_base_init;

    // **读取 gold_floating_base（world -> base）**
    Eigen::Vector3f pos_world_base_gold(gold_floating_base_yaml["gold_floating_base"][0].as<float>(), gold_floating_base_yaml["gold_floating_base"][1].as<float>(), gold_floating_base_yaml["gold_floating_base"][2].as<float>());
    Eigen::Matrix3f rot_world_base_gold =
        quaternionToRotationMatrix(gold_floating_base_yaml["gold_floating_base"][3].as<float>(), gold_floating_base_yaml["gold_floating_base"][4].as<float>(), gold_floating_base_yaml["gold_floating_base"][5].as<float>(), gold_floating_base_yaml["gold_floating_base"][6].as<float>());
    Eigen::Matrix4f gold_tf_mat_world_base = Eigen::Matrix4f::Identity();
    gold_tf_mat_world_base.block<3, 3>(0, 0) = rot_world_base_gold;
    gold_tf_mat_world_base.block<3, 1>(0, 3) = pos_world_base_gold;

    std::vector<Eigen::Matrix4f> interpolated_poses;
    int point_per_stage = 250;
    int point = 1000;

    // 初始与目标位置/旋转
    Eigen::Vector3f pos_init = init_tf_mat_world_base.block<3, 1>(0, 3);
    Eigen::Vector3f pos_gold = gold_tf_mat_world_base.block<3, 1>(0, 3);
    Eigen::Matrix3f rot_init = init_tf_mat_world_base.block<3, 3>(0, 0);
    Eigen::Matrix3f rot_gold = gold_tf_mat_world_base.block<3, 3>(0, 0);
    Eigen::Quaternionf q_init(rot_init);
    Eigen::Quaternionf q_gold(rot_gold);

    // 当前状态初始化
    Eigen::Vector3f pos_current = pos_init;
    Eigen::Quaternionf q_current = q_init;

    // 阶段1：插值 Z
    for (int i = 0; i < point_per_stage; ++i)
    {
        float t = static_cast<float>(i) / (point_per_stage - 1);
        pos_current.z() = (1 - t) * pos_init.z() + t * pos_gold.z();

        Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
        tf.block<3, 3>(0, 0) = q_current.toRotationMatrix();
        tf.block<3, 1>(0, 3) = pos_current;
        interpolated_poses.push_back(tf);
    }

    // 阶段2：插值 X
    for (int i = 0; i < point_per_stage; ++i)
    {
        float t = static_cast<float>(i) / (point_per_stage - 1);
        pos_current.x() = (1 - t) * pos_init.x() + t * pos_gold.x();

        Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
        tf.block<3, 3>(0, 0) = q_current.toRotationMatrix();
        tf.block<3, 1>(0, 3) = pos_current;
        interpolated_poses.push_back(tf);
    }

    // 阶段3：插值 Y
    for (int i = 0; i < point_per_stage; ++i)
    {
        float t = static_cast<float>(i) / (point_per_stage - 1);
        pos_current.y() = (1 - t) * pos_init.y() + t * pos_gold.y();

        Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
        tf.block<3, 3>(0, 0) = q_current.toRotationMatrix();
        tf.block<3, 1>(0, 3) = pos_current;
        interpolated_poses.push_back(tf);
    }

    // 阶段4：插值旋转（SLERP）
    for (int i = 0; i < point_per_stage; ++i)
    {
        float t = static_cast<float>(i) / (point_per_stage - 1);
        q_current = q_init.slerp(t, q_gold);

        Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
        tf.block<3, 3>(0, 0) = q_current.toRotationMatrix();
        tf.block<3, 1>(0, 3) = pos_current;  // 平移保持最终状态
        interpolated_poses.push_back(tf);
    }

    // **读取 TF 变换**
    Eigen::Matrix4f tf_mat_world_flan1 = parseTransformMatrix(tf_using_yaml["tf_mat_world_flan1"]);
    Eigen::Matrix4f tf_mat_world_flan4 = parseTransformMatrix(tf_using_yaml["tf_mat_world_flan4"]);
    Eigen::Matrix4f tf_mat_base_link1_0 = parseTransformMatrix(tf_using_yaml["tf_mat_base_link1_0"]);
    Eigen::Matrix4f tf_mat_base_link4_0 = parseTransformMatrix(tf_using_yaml["tf_mat_base_link4_0"]);

    std::vector<Eigen::Matrix4f> all_tf_link1_0_flan1;
    std::vector<Eigen::Matrix4f> all_tf_link4_0_flan4;

    // **遍历所有插值矩阵，计算 flan1 和 flan4 的变换**
    for (int i = 0; i < point; ++i)
    {
        const Eigen::Matrix4f& tf_interp = interpolated_poses[i];

        // 计算 base 坐标系下 flan1 和 flan4
        Eigen::Matrix4f tf_mat_base_flan1 = tf_interp.inverse() * tf_mat_world_flan1;
        Eigen::Matrix4f tf_mat_base_flan4 = tf_interp.inverse() * tf_mat_world_flan4;

        // 计算 link1_0 下的 flan1
        Eigen::Matrix4f tf_mat_link1_0_flan1 = tf_mat_base_link1_0.inverse() * tf_mat_base_flan1;

        // 计算 link4_0 下的 flan4
        Eigen::Matrix4f tf_mat_link4_0_flan4 = tf_mat_base_link4_0.inverse() * tf_mat_base_flan4;

        // 将结果存储到向量中
        all_tf_link1_0_flan1.push_back(tf_mat_link1_0_flan1);
        all_tf_link4_0_flan4.push_back(tf_mat_link4_0_flan4);
    }

    // **保存计算结果到 YAML**
    saveToYAML(output_file, all_tf_link1_0_flan1, all_tf_link4_0_flan4);

    ROS_INFO("All computed transformations saved to leg_ik_cs.yaml!");
    return true;
}

} // namespace robot_planning
