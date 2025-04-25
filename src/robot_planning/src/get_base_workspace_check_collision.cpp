#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <random>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include "robot_planning/ikfast_wrapper_single_arm.h"
#include <ros/package.h>
#include <ros/service.h>
#include "robot_rrt/CheckCollision.h"
#include <sensor_msgs/JointState.h>

enum JointID : int
{
    // 先放平台关节
    JOINT_PLATFORM = 0,

    // Branch 1
    JOINT1_0,
    JOINT1_1,
    JOINT1_2,
    JOINT1_3,
    JOINT1_4,
    JOINT1_5,
    JOINT1_6,
    LINK1_FINGER,

    // Branch 2
    JOINT2_0,
    JOINT2_1,
    JOINT2_2,
    JOINT2_3,
    JOINT2_4,
    JOINT2_5,
    JOINT2_6,
    LINK2_FINGER,

    // Branch 3
    JOINT3_0,
    JOINT3_1,
    JOINT3_2,
    JOINT3_3,
    JOINT3_4,
    JOINT3_5,
    JOINT3_6,
    LINK3_FINGER,

    // Branch 4
    JOINT4_0,
    JOINT4_1,
    JOINT4_2,
    JOINT4_3,
    JOINT4_4,
    JOINT4_5,
    JOINT4_6,
    LINK4_FINGER,

    JOINT1_PLATLINK,
    JOINT2_PLATLINK,
    JOINT3_PLATLINK,
    JOINT4_PLATLINK,

    // 用来标记总数量，必须放在最后
    JOINT_COUNT
};

void fillFixedJointValues(std::vector<double>& joint_positions)
{
    joint_positions.resize(JOINT_COUNT, 0.0);

    joint_positions[JOINT1_0] = 1.5708;
    joint_positions[JOINT2_0] = 1.5708;
    joint_positions[JOINT3_0] = 1.5708;
    joint_positions[JOINT4_0] = 1.5708;

    joint_positions[LINK1_FINGER] = 0.5;
    joint_positions[LINK2_FINGER] = 0.0;
    joint_positions[LINK3_FINGER] = 0.0;
    joint_positions[LINK4_FINGER] = 0.5;

    joint_positions[JOINT_PLATFORM] = 0.077888;
    joint_positions[JOINT1_PLATLINK] = -0.847454;
    joint_positions[JOINT2_PLATLINK] = -0.847454;
    joint_positions[JOINT3_PLATLINK] = -0.847454;
    joint_positions[JOINT4_PLATLINK] = -0.847454;

    // 填写 branch2 关节角
    joint_positions[JOINT2_1] = -1.18542;
    joint_positions[JOINT2_2] = -1.84009;
    joint_positions[JOINT2_3] = -1.90918;
    joint_positions[JOINT2_4] = 1.5428;
    joint_positions[JOINT2_5] = 1.18639;
    joint_positions[JOINT2_6] = -2.28165;

    // 填写 branch3 关节角
    joint_positions[JOINT3_1] = 1.20091;
    joint_positions[JOINT3_2] = -1.84009;
    joint_positions[JOINT3_3] = -1.90918;
    joint_positions[JOINT3_4] = -1.54403;
    joint_positions[JOINT3_5] = 1.20184;
    joint_positions[JOINT3_6] = -0.859485;
}

// 将旋转矩阵和平移向量转换为12维pose向量
std::vector<float> transformToVector(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans)
{
    std::vector<float> pose(12);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            pose[i * 4 + j] = rot(i, j);
        }
        pose[i * 4 + 3] = trans(i);
    }
    return pose;
}

// 从YAML中读取变换矩阵
void readTransformFromYAML(const YAML::Node& node, Eigen::Matrix4d& transform)
{
    auto position = node["target_pose"]["position"].as<std::vector<double>>();
    auto orientation = node["target_pose"]["orientation"];
    transform.setIdentity();
    for (int i = 0; i < 3; i++)
    {
        auto row = orientation[i].as<std::vector<double>>();
        for (int j = 0; j < 3; j++)
        {
            transform(i, j) = row[j];
        }
        transform(i, 3) = position[i];
    }
}

// 保存可行的 base 位姿（flan1 → base）到 YAML 文件
void saveBasePosesToYAML(const std::vector<Eigen::Matrix4d>& poses, const std::string& filename)
{
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        const auto& T = poses[i];
        Eigen::Vector3d t = T.block<3, 1>(0, 3);
        Eigen::Quaterniond q(T.block<3, 3>(0, 0));

        out << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << static_cast<int>(i + 1);
        out << YAML::Key << "position" << YAML::Value << YAML::Flow << std::vector<double>{t.x(), t.y(), t.z()};
        out << YAML::Key << "orientation" << YAML::Value << YAML::Flow << std::vector<double>{q.w(), q.x(), q.y(), q.z()};
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    std::ofstream fout(filename);
    if (fout.is_open())
    {
        fout << out.c_str();
        fout.close();
        ROS_INFO("Saved feasible base poses with IDs to: %s", filename.c_str());
    }
    else
    {
        ROS_ERROR("Failed to write YAML file: %s", filename.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_base_workspace");
    ros::NodeHandle nh;

    // ==== 读取配置文件 ====
    std::string save_path = ros::package::getPath("robot_planning") + "/config/workspace.yaml";
    YAML::Node config = YAML::LoadFile(save_path);

    Eigen::Matrix4d T_base_link1_0, T_base_link4_0, T_flan1_flan4;
    readTransformFromYAML(config["tf_mat_base_link1_0"], T_base_link1_0);
    readTransformFromYAML(config["tf_mat_base_link4_0"], T_base_link4_0);
    readTransformFromYAML(config["tf_mat_flan1_flan4"], T_flan1_flan4);

    // ==== 创建IK求解器 ====
    robots::Kinematics ik_solver;

    // ==== 定义关节限位 ====
    std::vector<std::pair<float, float>> joint_limits = {{-2.3562f, 4.7124f}, {-2.0944f, 1.5708f}, {-2.5307f, 2.5307f}, {-3.1416f, 3.1416f}, {-1.9199f, 1.9199f}, {-3.1416f, 3.1416f}};

    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<std::uniform_real_distribution<>> distributions;
    for (const auto& lim : joint_limits)
    {
        distributions.emplace_back(lim.first, lim.second);
    }

    std::vector<Eigen::Matrix4d> feasible_base_poses;
    const int NUM_SAMPLES = 1000000;
    for (int i = 0; i < NUM_SAMPLES; ++i)
    {
        // ==== Step 1: 对分支1进行随机采样 ====
        std::vector<float> joint_config1;
        for (int j = 0; j < 6; ++j)
        {
            joint_config1.push_back(distributions[j](gen));
        }

        // ==== Step 2: 正向运动学获得 flan1 相对于 link1_0 的位姿 ====
        std::vector<float> ee_pose1 = ik_solver.forward(joint_config1);
        if (ee_pose1.empty())
            continue;

        Eigen::Matrix4d T_ee1 = Eigen::Matrix4d::Identity();
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                T_ee1(r, c) = ee_pose1[r * 4 + c];
            }
            T_ee1(r, 3) = ee_pose1[r * 4 + 3];
        }

        // ==== Step 3: flan1 → base（用在flan1坐标系下描述base）
        Eigen::Matrix4d T_base_in_flan1 = T_base_link1_0 * T_ee1;

        // ==== Step 4: 计算 flan4 在 link4_0 下的位姿
        Eigen::Matrix4d T_ee4 = T_base_link4_0.inverse() * T_base_in_flan1 * T_flan1_flan4;

        std::vector<float> target_pose4 = transformToVector(T_ee4.block<3, 3>(0, 0), T_ee4.block<3, 1>(0, 3));

        std::vector<float> solutions4 = ik_solver.inverse(target_pose4);

        // ==== Step 5: 若可达，保存 flan1 → base 的变换
        if (!solutions4.empty())
        {
            // 构造完整关节角
            std::vector<double> joint_positions;
            fillFixedJointValues(joint_positions);

            // 填入分支1的关节角
            for (int j = 0; j < 6; ++j) joint_positions[JOINT1_1 + j] = joint_config1[j];

            // 填入分支4的关节角（取 solutions4 的第一个解）
            for (int j = 0; j < 6; ++j) joint_positions[JOINT4_1 + j] = solutions4[j];

            // 发送服务请求
            robot_rrt::CheckCollision srv;
            srv.request.joint_positions = joint_positions;

            if (ros::service::call("/check_collision", srv))
            {
                if (!srv.response.collision)
                {
                    // 无碰撞，保存
                    Eigen::Matrix4d T_flan1_to_base = T_base_in_flan1.inverse();
                    feasible_base_poses.push_back(T_flan1_to_base);
                }
                // else 可以选择打印有碰撞信息
            }
            else
            {
                ROS_ERROR("Failed to call service /check_collision");
                break;  // 或 continue，根据需求处理
            }
        }
    }

    ROS_INFO("Total feasible base poses: %lu", feasible_base_poses.size());

    // ==== 保存结果到 YAML ====
    std::string output_path = ros::package::getPath("robot_planning") + "/config/feasible_base_poses.yaml";
    saveBasePosesToYAML(feasible_base_poses, output_path);

    return 0;
}
