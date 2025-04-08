#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <sstream>  // 用于数值格式化

// 辅助函数：格式化 double 数值，确保高精度
std::string format_double(double value, int precision = 2)
{
    std::ostringstream stream;
    stream.precision(precision);
    stream << std::fixed << value;
    return stream.str();
}

int main()
{
    const std::string urdf_filename = "../models/single_arm_pino.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    // 加载 fk_urdf.yaml
    YAML::Node config = YAML::LoadFile("../config/fk_urdf.yaml");

    YAML::Node config_ik;
    try
    {
        config_ik = YAML::LoadFile("../config/ik_urdf.yaml");
    }
    catch (...)
    {
        std::cerr << "Warning: 'ik_urdf.yaml' not found, creating new one." << std::endl;
    }

    Eigen::VectorXd q(model.nq);
    if (config["joint_angles"] && config["joint_angles"].IsSequence())
    {
        for (std::size_t i = 0; i < model.nq; ++i)
        {
            q[i] = config["joint_angles"][i].as<double>();
        }
    }
    else
    {
        std::cerr << "Error: 'joint_angles' is missing in fk_urdf.yaml!" << std::endl;
        return -1;
    }

    std::cout << "Joint configuration from YAML: " << q.transpose() << std::endl;

    // 计算正向运动学
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    const int JOINT_ID = model.getJointId("Joint1_1");  // 末端执行器的 joint name

    // 获取末端执行器位姿
    pinocchio::SE3 end_effector_pose = data.oMi[JOINT_ID];

    std::cout << "End-effector pose:\n" << end_effector_pose << std::endl;

    // 转换为齐次矩阵
    Eigen::Matrix4d T = end_effector_pose.toHomogeneousMatrix();
    std::cout << "End-effector pose (T):\n" << T << std::endl;

    // Save end-effector pose to YAML file
    YAML::Emitter out;
    out << YAML::BeginMap;

    // Read previous init_joint_angles if exists
    std::vector<double> init_angles;
    if (config_ik["init_joint_angles"] && config_ik["init_joint_angles"].IsSequence())
    {
        for (const auto& val : config_ik["init_joint_angles"])
        {
            init_angles.push_back(val.as<double>());
        }
    }
    else
    {
        std::cerr << "Warning: 'init_joint_angles' not found in ik_urdf.yaml, using current joint angles." << std::endl;
        for (int i = 0; i < model.nq; ++i)
        {
            init_angles.push_back(q[i]);
        }
    }

    std::cout << "Before writing YAML: init_joint_angles = ";
    for (double angle : init_angles)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    out << YAML::Key << "init_joint_angles" << YAML::Value << YAML::Flow << init_angles;

    out << YAML::Key << "target_pose" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "position" << YAML::Value << YAML::Flow << std::vector<double>{end_effector_pose.translation()[0], end_effector_pose.translation()[1], end_effector_pose.translation()[2]};
    out << YAML::Key << "orientation" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i)
    {
        out << YAML::Flow << std::vector<double>{end_effector_pose.rotation()(i, 0), end_effector_pose.rotation()(i, 1), end_effector_pose.rotation()(i, 2)};
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    out << YAML::EndMap;

    std::ofstream fout("../config/ik_urdf.yaml");
    fout << out.c_str();
    fout.close();

    return 0;
}
