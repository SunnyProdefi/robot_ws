#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>
#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

// 角度转弧度
#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)

void saveIKResult(const std::string& filename, const std::vector<double>& qIk, int floating_base_size, int total_joints)
{
    if (qIk.size() != total_joints)
    {
        throw std::runtime_error("qIk size does not match total_joints!");
    }

    std::ofstream output_file(filename, std::ios::out | std::ios::trunc);  // 以截断模式写入，确保每次保存最新的数据
    if (!output_file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    output_file << "floating_base: [";
    for (int i = 0; i < floating_base_size; ++i)
    {
        output_file << qIk[i] << (i < floating_base_size - 1 ? ", " : "");
    }
    output_file << "]\n";

    output_file << "joint_angles: [";
    for (int i = floating_base_size; i < total_joints; ++i)
    {
        output_file << qIk[i] << (i < total_joints - 1 ? ", " : "");
    }
    output_file << "]\n";

    output_file.close();
}

int main()
{
    const std::string urdf_filename = "../models/double_arms_description.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model);
    pinocchio::Data data(model);

    // 打印模型信息
    std::cout << "Model has " << model.nq << " DOFs and " << model.nv << " velocity DOFs." << std::endl;

    // 打印关节限位
    std::cout << "Joint limits: " << std::endl;
    for (int i = 0; i < model.nq - 5; ++i)
    {
        std::cout << model.names[i] << ": [" << model.lowerPositionLimit[i + 5] << ", " << model.upperPositionLimit[i + 5] << "]" << std::endl;
    }

    // 读取 YAML 文件
    YAML::Node config_joint_limits = YAML::LoadFile("../config/joint_limits.yaml");

    // 读取角度限位
    Eigen::VectorXd angle_min(12), angle_max(12);
    for (int i = 0; i < 6; ++i)
    {
        std::string joint_name_2 = "Joint2_" + std::to_string(i + 1);
        std::string joint_name_3 = "Joint3_" + std::to_string(i + 1);

        if (config_joint_limits["joint_limits"][joint_name_2] && config_joint_limits["joint_limits"][joint_name_2]["angle"] && config_joint_limits["joint_limits"][joint_name_2]["angle"].size() >= 2)
        {
            angle_min[i] = config_joint_limits["joint_limits"][joint_name_2]["angle"][0].as<double>();
            angle_max[i] = config_joint_limits["joint_limits"][joint_name_2]["angle"][1].as<double>();
        }
        else
        {
            std::cerr << "Error: Missing or incorrect angle data for " << joint_name_2 << std::endl;
        }

        if (config_joint_limits["joint_limits"][joint_name_3] && config_joint_limits["joint_limits"][joint_name_3]["angle"] && config_joint_limits["joint_limits"][joint_name_3]["angle"].size() >= 2)
        {
            angle_min[i + 6] = config_joint_limits["joint_limits"][joint_name_3]["angle"][0].as<double>();
            angle_max[i + 6] = config_joint_limits["joint_limits"][joint_name_3]["angle"][1].as<double>();
        }
        else
        {
            std::cerr << "Error: Missing or incorrect angle data for " << joint_name_3 << std::endl;
        }
    }
    // 打印结果
    std::cout << "Joint Angle Min: " << angle_min.transpose() << std::endl;
    std::cout << "Joint Angle Max: " << angle_max.transpose() << std::endl;

    // 定义浮动关节限位
    Eigen::VectorXd floating_min(7), floating_max(7);

    if (config_joint_limits["floating_joint"])
    {
        // 读取位置信息 (x, y, z)
        if (config_joint_limits["floating_joint"]["position"])
        {
            floating_min.head<3>() = Eigen::Vector3d(config_joint_limits["floating_joint"]["position"]["min"][0].as<double>(), config_joint_limits["floating_joint"]["position"]["min"][1].as<double>(), config_joint_limits["floating_joint"]["position"]["min"][2].as<double>());

            floating_max.head<3>() = Eigen::Vector3d(config_joint_limits["floating_joint"]["position"]["max"][0].as<double>(), config_joint_limits["floating_joint"]["position"]["max"][1].as<double>(), config_joint_limits["floating_joint"]["position"]["max"][2].as<double>());
        }
        else
        {
            std::cerr << "Warning: Missing 'position' limits in 'floating_joint'!" << std::endl;
        }

        // 读取旋转信息 (欧拉角 -> 转换为四元数)
        if (config_joint_limits["floating_joint"]["rotation"])
        {
            Eigen::Vector3d euler_min, euler_max;

            euler_min = Eigen::Vector3d(DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_min"][0].as<double>()),   // X 轴
                                        DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_min"][1].as<double>()),   // Y 轴
                                        DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_min"][2].as<double>()));  // Z 轴

            euler_max = Eigen::Vector3d(DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_max"][0].as<double>()),   // X 轴
                                        DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_max"][1].as<double>()),   // Y 轴
                                        DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_max"][2].as<double>()));  // Z 轴

            // 计算最小旋转的四元数
            Eigen::Quaterniond quat_min;
            quat_min = Eigen::AngleAxisd(euler_min[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler_min[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler_min[2], Eigen::Vector3d::UnitZ());

            // 计算最大旋转的四元数
            Eigen::Quaterniond quat_max;
            quat_max = Eigen::AngleAxisd(euler_max[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler_max[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler_max[2], Eigen::Vector3d::UnitZ());

            // 存入 floating_min 和 floating_max
            floating_min.tail<4>() << quat_min.x(), quat_min.y(), quat_min.z(), quat_min.w();
            floating_max.tail<4>() << quat_max.x(), quat_max.y(), quat_max.z(), quat_max.w();
        }
        else
        {
            std::cerr << "Warning: Missing 'rotation' limits in 'floating_joint'!" << std::endl;
        }
    }
    else
    {
        std::cerr << "Error: 'floating_joint' not found in YAML file!" << std::endl;
    }

    // 打印结果
    std::cout << "Floating Joint Min: " << floating_min.transpose() << std::endl;
    std::cout << "Floating Joint Max: " << floating_max.transpose() << std::endl;

    // 读取初始关节角度 & 目标位姿
    YAML::Node config = YAML::LoadFile("../config/ik_urdf_double_arm_float.yaml");
    Eigen::VectorXd qIk(model.nq);
    for (std::size_t i = 0; i < 7; ++i) qIk[i] = config["init_floating_base"][i].as<double>();
    for (std::size_t i = 0; i < model.nq - 7; ++i) qIk[i + 7] = config["init_joint_angles"][i].as<double>();

    std::cout << "Initial joint config: " << qIk.transpose() << std::endl;

    Eigen::Vector3d target_position_R(config["target_pose_R"]["position"][0].as<double>(), config["target_pose_R"]["position"][1].as<double>(), config["target_pose_R"]["position"][2].as<double>());

    Eigen::Matrix3d target_orientation_R;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            target_orientation_R(i, j) = config["target_pose_R"]["orientation"][i][j].as<double>();
        }
    }

    Eigen::Vector3d target_position_L(config["target_pose_L"]["position"][0].as<double>(), config["target_pose_L"]["position"][1].as<double>(), config["target_pose_L"]["position"][2].as<double>());

    Eigen::Matrix3d target_orientation_L;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            target_orientation_L(i, j) = config["target_pose_L"]["orientation"][i][j].as<double>();
        }
    }

    pinocchio::SE3 oMdes_R(target_orientation_R, target_position_R);
    pinocchio::SE3 oMdes_L(target_orientation_L, target_position_L);

    std::cout << "Target pose R:\n" << oMdes_R << std::endl;
    std::cout << "Target pose L:\n" << oMdes_L << std::endl;

    // **IK 超参数**
    const int JOINT_ID_R = model.getJointId("Joint3_6");  // 右臂末端执行器关节索引
    const int JOINT_ID_L = model.getJointId("Joint2_6");  // 左臂末端执行器关节索引

    const double eps = 1e-3;   // 收敛阈值
    const int IT_MAX = 50000;  // 最大迭代次数
    const double DT = 6e-1;    // 更新步长
    const double damp = 1e-2;  // 阻尼因子

    // **存储变量**
    pinocchio::Data::Matrix6x J_R(6, model.nv), J_L(6, model.nv);
    J_R.setZero();
    J_L.setZero();

    Eigen::Matrix<double, 6, 1> err_R, err_L;
    Eigen::Matrix<double, 12, 1> err_stacked;
    Eigen::VectorXd v(model.nv);

    bool success = false;
    int iter_count = 0;

    // **迭代计算**
    for (iter_count = 0;; ++iter_count)
    {
        pinocchio::forwardKinematics(model, data, qIk);
        pinocchio::updateFramePlacements(model, data);

        // **计算误差**
        pinocchio::SE3 iMd_R = data.oMi[JOINT_ID_R].actInv(oMdes_R);
        pinocchio::SE3 iMd_L = data.oMi[JOINT_ID_L].actInv(oMdes_L);
        err_R = pinocchio::log6(iMd_R).toVector();
        err_L = pinocchio::log6(iMd_L).toVector();
        if (!(iter_count % 1000))
        {
            std::cout << "[Iteration " << iter_count << "] Error_L: " << err_L.norm() << " Error_R: " << err_R.norm() << std::endl;
        }

        // **收敛判断**
        if (err_L.norm() < eps && err_R.norm() < eps)
        {
            success = true;
            break;
        }
        if (iter_count >= IT_MAX)
        {
            success = false;
            break;
        }

        // **计算雅可比矩阵**
        pinocchio::computeJointJacobian(model, data, qIk, JOINT_ID_R, J_R);
        pinocchio::computeJointJacobian(model, data, qIk, JOINT_ID_L, J_L);

        pinocchio::Data::Matrix6 Jlog_R, Jlog_L;
        pinocchio::Jlog6(iMd_R.inverse(), Jlog_R);
        pinocchio::Jlog6(iMd_L.inverse(), Jlog_L);

        J_R = -Jlog_R * J_R;
        J_L = -Jlog_L * J_L;

        // **阻尼求解**
        pinocchio::Data::Matrix6 JJt_L, JJt_R;
        JJt_L.noalias() = J_L * J_L.transpose();
        JJt_R.noalias() = J_R * J_R.transpose();
        JJt_L.diagonal().array() += damp;
        JJt_R.diagonal().array() += damp;

        Eigen::VectorXd v_L = -J_L.transpose() * JJt_L.ldlt().solve(err_L);
        Eigen::VectorXd v_R = -J_R.transpose() * JJt_R.ldlt().solve(err_R);
        v = (v_L + v_R) / 2.0;  // **平均两个机械臂的优化结果**

        // **更新关节角度**
        qIk = pinocchio::integrate(model, qIk, v * DT);

        // **添加浮动关节限位**
        for (int i = 0; i < 7; ++i)  // 限制 x, y, z 和四元数
        {
            qIk[i] = std::max(floating_min[i], std::min(qIk[i], floating_max[i]));
        }
        // **归一化四元数**
        Eigen::Quaterniond q(qIk[6], qIk[3], qIk[4], qIk[5]);  // 读取四元数
        q.normalize();                                         // 归一化
        qIk[3] = q.x();
        qIk[4] = q.y();
        qIk[5] = q.z();
        qIk[6] = q.w();

        // **添加关节限位检查**
        for (int i = 7; i < model.nq; ++i)
        {
            if (qIk[i] < angle_min[i - 7])
            {
                qIk[i] = angle_min[i - 7];
            }
            else if (qIk[i] > angle_max[i - 7])
            {
                qIk[i] = angle_max[i - 7];
            }
        }

        // **打印调试信息**
        if (!(iter_count % 1000))
        {
            std::cout << "[Iteration " << iter_count << "] Error_L: " << err_L.transpose() << " Error_R: " << err_R.transpose() << std::endl;
        }
    }

    // 判断是否收敛
    if (success)
    {
        std::cout << "[Double Arm IK] Converged in " << iter_count << " iterations.\n";
    }
    else
    {
        std::cout << "[Double Arm IK] Did NOT converge after " << IT_MAX << " iterations.\n";
    }

    // 输出最终结果
    std::cout << "Final joint configuration: " << qIk.transpose() << std::endl;

    // 判断是否超出关节限位
    for (int i = 7; i < model.nq; ++i)
    {
        if (qIk[i] < angle_min[i - 7])
        {
            std::cout << "Joint " << model.names[i - 5] << " exceeds lower limit: " << qIk[i] << " < " << angle_min[i - 7] << std::endl;
        }
        else if (qIk[i] > angle_max[i - 7])
        {
            std::cout << "Joint " << model.names[i - 5] << " exceeds upper limit: " << qIk[i] << " > " << angle_max[i - 7] << std::endl;
        }
    }

    // 输出最终结果到文件
    std::vector<double> qIk_vec(model.nq);
    for (int i = 0; i < model.nq; ++i) qIk_vec[i] = qIk[i];
    try
    {
        saveIKResult("../config/ik_urdf_double_arm_float_result.yaml", qIk_vec, 7, model.nq);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // 做一次前向运动学以验证末端结果
    pinocchio::forwardKinematics(model, data, qIk);
    pinocchio::updateFramePlacements(model, data);

    pinocchio::SE3 finalPose_R = data.oMi[JOINT_ID_R];
    pinocchio::SE3 finalPose_L = data.oMi[JOINT_ID_L];
    std::cout << "Right End-Effector pose:\n" << finalPose_R << std::endl;
    std::cout << "Left  End-Effector pose:\n" << finalPose_L << std::endl;

    return 0;
}
