#include "robot_planning/ik_solver.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)

namespace robot_planning {

IKSolver::IKSolver(const IKSolverConfig& config) : config_(config) {
    // 加载URDF模型
    pinocchio::urdf::buildModel(config.urdf_path, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);

    // 读取关节限位
    YAML::Node config_joint_limits = YAML::LoadFile(config.joint_limits_path);
    
    // 初始化角度限位
    angle_min_.resize(12);
    angle_max_.resize(12);
    for (int i = 0; i < 6; ++i) {
        std::string joint_name_2 = "Joint2_" + std::to_string(i + 1);
        std::string joint_name_3 = "Joint3_" + std::to_string(i + 1);

        if (config_joint_limits["joint_limits"][joint_name_2] && 
            config_joint_limits["joint_limits"][joint_name_2]["angle"] && 
            config_joint_limits["joint_limits"][joint_name_2]["angle"].size() >= 2) {
            angle_min_[i] = config_joint_limits["joint_limits"][joint_name_2]["angle"][0].as<double>();
            angle_max_[i] = config_joint_limits["joint_limits"][joint_name_2]["angle"][1].as<double>();
        }

        if (config_joint_limits["joint_limits"][joint_name_3] && 
            config_joint_limits["joint_limits"][joint_name_3]["angle"] && 
            config_joint_limits["joint_limits"][joint_name_3]["angle"].size() >= 2) {
            angle_min_[i + 6] = config_joint_limits["joint_limits"][joint_name_3]["angle"][0].as<double>();
            angle_max_[i + 6] = config_joint_limits["joint_limits"][joint_name_3]["angle"][1].as<double>();
        }
    }

    // 初始化浮动关节限位
    floating_min_.resize(7);
    floating_max_.resize(7);

    if (config_joint_limits["floating_joint"]) {
        // 读取位置信息
        if (config_joint_limits["floating_joint"]["position"]) {
            floating_min_.head<3>() = Eigen::Vector3d(
                config_joint_limits["floating_joint"]["position"]["min"][0].as<double>(),
                config_joint_limits["floating_joint"]["position"]["min"][1].as<double>(),
                config_joint_limits["floating_joint"]["position"]["min"][2].as<double>()
            );
            floating_max_.head<3>() = Eigen::Vector3d(
                config_joint_limits["floating_joint"]["position"]["max"][0].as<double>(),
                config_joint_limits["floating_joint"]["position"]["max"][1].as<double>(),
                config_joint_limits["floating_joint"]["position"]["max"][2].as<double>()
            );
        }

        // 读取旋转信息
        if (config_joint_limits["floating_joint"]["rotation"]) {
            Eigen::Vector3d euler_min = Eigen::Vector3d(
                DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_min"][0].as<double>()),
                DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_min"][1].as<double>()),
                DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_min"][2].as<double>())
            );
            Eigen::Vector3d euler_max = Eigen::Vector3d(
                DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_max"][0].as<double>()),
                DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_max"][1].as<double>()),
                DEG2RAD(config_joint_limits["floating_joint"]["rotation"]["euler_max"][2].as<double>())
            );

            Eigen::Quaterniond quat_min = Eigen::AngleAxisd(euler_min[0], Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(euler_min[1], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(euler_min[2], Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond quat_max = Eigen::AngleAxisd(euler_max[0], Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(euler_max[1], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(euler_max[2], Eigen::Vector3d::UnitZ());

            floating_min_.tail<4>() << quat_min.x(), quat_min.y(), quat_min.z(), quat_min.w();
            floating_max_.tail<4>() << quat_max.x(), quat_max.y(), quat_max.z(), quat_max.w();
        }
    }
}

IKSolverResult IKSolver::solveIK(const Eigen::Vector3d& target_pos_R,
                                const Eigen::Matrix3d& target_ori_R,
                                const Eigen::Vector3d& target_pos_L,
                                const Eigen::Matrix3d& target_ori_L) {
    IKSolverResult result;
    
    // 读取初始配置
    YAML::Node config = YAML::LoadFile(config_.init_config_path);
    Eigen::VectorXd qIk(model_.nq);
    for (std::size_t i = 0; i < 7; ++i) qIk[i] = config["init_floating_base"][i].as<double>();
    for (std::size_t i = 0; i < model_.nq - 7; ++i) qIk[i + 7] = config["init_joint_angles"][i].as<double>();

    // 设置目标位姿
    pinocchio::SE3 oMdes_R(target_ori_R, target_pos_R);
    pinocchio::SE3 oMdes_L(target_ori_L, target_pos_L);

    // 获取关节ID
    const int JOINT_ID_R = model_.getJointId("Joint3_6");
    const int JOINT_ID_L = model_.getJointId("Joint2_6");

    // 初始化变量
    pinocchio::Data::Matrix6x J_R(6, model_.nv), J_L(6, model_.nv);
    J_R.setZero();
    J_L.setZero();

    Eigen::Matrix<double, 6, 1> err_R, err_L;
    Eigen::Matrix<double, 12, 1> err_stacked;
    Eigen::VectorXd v(model_.nv);

    // 迭代求解
    int iter_count = 0;
    for (; iter_count < config_.max_iterations; ++iter_count) {
        pinocchio::forwardKinematics(model_, data_, qIk);
        pinocchio::updateFramePlacements(model_, data_);

        // 计算误差
        pinocchio::SE3 iMd_R = data_.oMi[JOINT_ID_R].actInv(oMdes_R);
        pinocchio::SE3 iMd_L = data_.oMi[JOINT_ID_L].actInv(oMdes_L);
        err_R = pinocchio::log6(iMd_R).toVector();
        err_L = pinocchio::log6(iMd_L).toVector();

        // 收敛判断
        if (err_L.norm() < config_.eps && err_R.norm() < config_.eps) {
            result.success = true;
            break;
        }

        // 计算雅可比矩阵
        pinocchio::computeJointJacobian(model_, data_, qIk, JOINT_ID_R, J_R);
        pinocchio::computeJointJacobian(model_, data_, qIk, JOINT_ID_L, J_L);

        pinocchio::Data::Matrix6 Jlog_R, Jlog_L;
        pinocchio::Jlog6(iMd_R.inverse(), Jlog_R);
        pinocchio::Jlog6(iMd_L.inverse(), Jlog_L);

        J_R = -Jlog_R * J_R;
        J_L = -Jlog_L * J_L;

        // 阻尼求解
        err_stacked << err_R, err_L;
        Eigen::MatrixXd J_stacked(12, model_.nv);
        J_stacked << J_R, J_L;

        Eigen::MatrixXd JJt = J_stacked * J_stacked.transpose();
        JJt.diagonal().array() += config_.damp * config_.damp;
        v = -J_stacked.transpose() * JJt.ldlt().solve(err_stacked);

        // 更新关节角度
        qIk = pinocchio::integrate(model_, qIk, v * config_.dt);

        // 应用关节限位
        for (int i = 0; i < 7; ++i) {
            qIk[i] = std::max(floating_min_[i], std::min(floating_max_[i], qIk[i]));
        }
        for (int i = 7; i < model_.nq; ++i) {
            qIk[i] = std::max(angle_min_[i-7], std::min(angle_max_[i-7], qIk[i]));
        }
    }

    if (iter_count >= config_.max_iterations) {
        result.success = false;
        result.message = "IK solver failed to converge";
    } else {
        result.success = true;
        result.message = "IK solver converged successfully";
        result.joint_angles = std::vector<double>(qIk.data(), qIk.data() + qIk.size());
    }

    return result;
}

void IKSolver::saveResult(const std::string& filename, const std::vector<double>& qIk) {
    if (qIk.size() != model_.nq) {
        throw std::runtime_error("qIk size does not match model.nq!");
    }

    std::ofstream output_file(filename, std::ios::out | std::ios::trunc);
    if (!output_file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    output_file << "floating_base: [";
    for (int i = 0; i < 7; ++i) {
        output_file << qIk[i] << (i < 6 ? ", " : "");
    }
    output_file << "]\n";

    output_file << "joint_angles: [";
    for (int i = 7; i < model_.nq; ++i) {
        output_file << qIk[i] << (i < model_.nq - 1 ? ", " : "");
    }
    output_file << "]\n";

    output_file.close();
}

} // namespace robot_planning 