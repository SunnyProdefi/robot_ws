#include "robot_mpc/mpc_controller.h"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/aba.hpp>  // aba
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>

#include <iostream>

using namespace std;

MpcController::MpcController(const std::string& urdf_path, double delta_t, int horizon) : delta_t_(delta_t), horizon_(horizon)
{
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    dof_ = model_.nv;

    goal_state_.resize(2 * dof_);
    goal_state_.head(dof_) << 2.041711, -0.616538, 2.032447, -1.33452, 1.159544, -2.898303;
    goal_state_.tail(dof_) = Eigen::VectorXd::Zero(dof_);

    Q_ = Eigen::MatrixXd::Identity(2 * dof_ * horizon_, 2 * dof_ * horizon_) * 1000.0;
    W_ = Eigen::MatrixXd::Identity(dof_ * horizon_, dof_ * horizon_) * 0.1;

    frame_id_ = model_.getFrameId("Link1_6");  // 假设末端执行器
}

void MpcController::ComputeDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qd)
{
    pinocchio::crba(model_, data_, q);
    H_ = data_.M;

    // 计算 C_
    pinocchio::rnea(model_, data_, q, qd, Eigen::VectorXd::Zero(dof_));
    C_ = data_.tau;

    // 计算 J_
    J_.resize(6, dof_);
    pinocchio::computeFrameJacobian(model_, data_, q, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_);
}

void MpcController::UpdateReferenceTrajectory(const Eigen::VectorXd& current_state)
{
    target_traj_.resize(2 * dof_, horizon_);

    // 当前状态
    Eigen::VectorXd q0 = current_state.head(dof_);
    Eigen::VectorXd qd0 = current_state.tail(dof_);

    // 目标状态
    Eigen::VectorXd qg = goal_state_.head(dof_);
    Eigen::VectorXd qdg = goal_state_.tail(dof_);

    for (int i = 0; i < horizon_; ++i)
    {
        double alpha = static_cast<double>(i + 1) / horizon_;  // 插值系数 ∈ (0, 1]

        Eigen::VectorXd q_interp = (1 - alpha) * q0 + alpha * qg;
        Eigen::VectorXd qd_interp = (1 - alpha) * qd0 + alpha * qdg;

        target_traj_.col(i).head(dof_) = q_interp;
        target_traj_.col(i).tail(dof_) = qd_interp;
    }
}

Eigen::VectorXd MpcController::SolveOnceMpc(const Eigen::VectorXd& q, const Eigen::VectorXd& qd)
{
    ComputeDynamics(q, qd);

    // ===== 1️⃣ 状态空间模型 =====
    Eigen::MatrixXd A_state = Eigen::MatrixXd::Identity(2 * dof_, 2 * dof_);
    A_state.block(0, dof_, dof_, dof_) = delta_t_ * Eigen::MatrixXd::Identity(dof_, dof_);

    Eigen::MatrixXd B_state = Eigen::MatrixXd::Zero(2 * dof_, dof_);
    B_state.block(dof_, 0, dof_, dof_) = H_.inverse() * delta_t_;

    Eigen::VectorXd C_state = Eigen::VectorXd::Zero(2 * dof_);
    C_state.tail(dof_) = -H_.inverse() * C_ * delta_t_;

    // ===== 2️⃣ 预测矩阵构造 =====
    int p = horizon_;  // 预测步长

    Eigen::MatrixXd Psi = Eigen::MatrixXd::Zero(2 * dof_ * p, 2 * dof_);
    for (int i = 0; i < p; ++i)
    {
        Psi.block(i * 2 * dof_, 0, 2 * dof_, 2 * dof_) = A_state;
        for (int j = 0; j < i; ++j)
        {
            Psi.block(i * 2 * dof_, 0, 2 * dof_, 2 * dof_) *= A_state;
        }
    }

    Eigen::MatrixXd Theta = Eigen::MatrixXd::Zero(2 * dof_ * p, dof_ * p);
    for (int i = 0; i < p; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            Theta.block(i * 2 * dof_, j * dof_, 2 * dof_, dof_) = B_state;
            for (int k = 0; k < (i - j); ++k)
            {
                Theta.block(i * 2 * dof_, j * dof_, 2 * dof_, dof_) = A_state * Theta.block(i * 2 * dof_, j * dof_, 2 * dof_, dof_);
            }
        }
    }

    Eigen::MatrixXd Xi_aug = Eigen::MatrixXd::Zero(2 * dof_ * p, p);
    for (int i = 0; i < p; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            Xi_aug.block(i * 2 * dof_, j, 2 * dof_, 1) = C_state;
            for (int k = 0; k < (i - j); ++k)
            {
                Xi_aug.block(i * 2 * dof_, j, 2 * dof_, 1) = A_state * Xi_aug.block(i * 2 * dof_, j, 2 * dof_, 1);
            }
        }
    }
    Eigen::VectorXd Xi = Xi_aug.rowwise().sum();

    // ===== 3️⃣ 构建目标轨迹 =====
    Eigen::VectorXd R(2 * dof_ * p);
    for (int i = 0; i < p; ++i)
    {
        if (i < target_traj_.cols())
            R.segment(i * 2 * dof_, 2 * dof_) = target_traj_.col(i);
        else
            R.segment(i * 2 * dof_, 2 * dof_) = target_traj_.col(target_traj_.cols() - 1);  // 重复最后一个
    }

    // ===== 4️⃣ 构建 QP 代价函数 =====
    Eigen::VectorXd x0(2 * dof_);
    x0 << q, qd;

    Eigen::VectorXd E = Psi * x0 + Xi - R;

    Eigen::MatrixXd H_qp = 2 * (Theta.transpose() * Q_ * Theta + W_);
    Eigen::VectorXd g_qp = 2 * Theta.transpose() * Q_ * E;

    // ===== 5️⃣ 变量约束 =====
    Eigen::VectorXd tau_lb = Eigen::VectorXd::Constant(dof_ * p, -100);  // 示例：τ下界
    Eigen::VectorXd tau_ub = Eigen::VectorXd::Constant(dof_ * p, 100);   // 示例：τ上界

    // ===== 6️⃣ 状态约束 =====
    Eigen::VectorXd X_lb = Eigen::VectorXd::Constant(2 * dof_ * p, -1e6);
    Eigen::VectorXd X_ub = Eigen::VectorXd::Constant(2 * dof_ * p, 1e6);
    // q_lower_.resize(dof_);
    // q_upper_.resize(dof_);
    // qd_lower_.resize(dof_);
    // qd_upper_.resize(dof_);

    // // 用真实的角度和速度限制替换以下内容
    // q_lower_ << -2.3562, -2.0944, -2.5307, -3.1416, -1.9199, -3.1416;
    // q_upper_ << 4.7124, 1.5708, 2.5307, 3.1416, 1.9199, 3.1416;

    // qd_lower_ = Eigen::VectorXd::Constant(dof_, -10.0);
    // qd_upper_ = Eigen::VectorXd::Constant(dof_, 10.0);

    // // 构建 X_lb, X_ub
    // Eigen::VectorXd X_lb = Eigen::VectorXd::Zero(2 * dof_ * p);
    // Eigen::VectorXd X_ub = Eigen::VectorXd::Zero(2 * dof_ * p);

    // for (int i = 0; i < p; ++i)
    // {
    //     X_lb.segment(i * 2 * dof_, dof_) = q_lower_;
    //     X_lb.segment(i * 2 * dof_ + dof_, dof_) = qd_lower_;
    //     X_ub.segment(i * 2 * dof_, dof_) = q_upper_;
    //     X_ub.segment(i * 2 * dof_ + dof_, dof_) = qd_upper_;
    // }

    Eigen::VectorXd A_lb = X_lb - Psi * x0 - Xi;
    Eigen::VectorXd A_ub = X_ub - Psi * x0 - Xi;

    // ===== 7️⃣ 使用 qpOASES 求解 =====
    USING_NAMESPACE_QPOASES
    QProblem qp(dof_ * p, 2 * dof_ * p);

    Options options;
    options.setToMPC();
    qp.setOptions(options);

    int nWSR = 100;
    qp.init(H_qp.data(), g_qp.data(), Theta.data(), tau_lb.data(), tau_ub.data(), A_lb.data(), A_ub.data(), nWSR);

    Eigen::VectorXd U_opt(dof_ * p);
    qp.getPrimalSolution(U_opt.data());

    std::cout << "===== [MPC Debug Info] =====" << std::endl;
    std::cout << "dof_: " << dof_ << ", horizon_: " << horizon_ << std::endl;

    std::cout << "A_state: " << A_state.rows() << "x" << A_state.cols() << std::endl;
    std::cout << "B_state: " << B_state.rows() << "x" << B_state.cols() << std::endl;
    std::cout << "C_state: " << C_state.transpose() << std::endl;

    std::cout << "Psi: " << Psi.rows() << "x" << Psi.cols() << std::endl;
    std::cout << "Theta: " << Theta.rows() << "x" << Theta.cols() << std::endl;
    std::cout << "Xi: " << Xi.rows() << "x" << Xi.cols() << std::endl;

    std::cout << "R: " << R.size() << std::endl;

    for (int i = 0; i < p; ++i)
    {
        int start_idx = i * 2 * dof_;
        if (start_idx + 2 * dof_ > R.size())
        {
            std::cerr << "[ERROR] R.segment(" << start_idx << ", " << 2 * dof_ << ") out of range! R.size() = " << R.size() << std::endl;
        }
    }

    return U_opt.head(dof_);  // 返回当前时刻 τ
}

void MpcController::RunMpcSimulation(const Eigen::VectorXd& init_state, int total_steps)
{
    Eigen::VectorXd q = init_state.head(dof_);
    Eigen::VectorXd qd = init_state.tail(dof_);
    Eigen::VectorXd qdd = Eigen::VectorXd::Zero(dof_);

    // YAML 初始化
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "trajectory_log" << YAML::Value << YAML::BeginSeq;

    for (int i = 0; i < total_steps; ++i)
    {
        Eigen::VectorXd current_state(2 * dof_);
        current_state.head(dof_) = q;
        current_state.tail(dof_) = qd;

        UpdateReferenceTrajectory(current_state);

        Eigen::VectorXd target = target_traj_.col(0);
        double error = (current_state - target).norm();

        // YAML记录当前步骤
        out << YAML::BeginMap;
        out << YAML::Key << "step" << YAML::Value << i;
        out << YAML::Key << "q" << YAML::Value << YAML::Flow << std::vector<double>(q.data(), q.data() + q.size());
        out << YAML::Key << "qd" << YAML::Value << YAML::Flow << std::vector<double>(qd.data(), qd.data() + qd.size());
        out << YAML::Key << "error" << YAML::Value << error;

        // 记录参考轨迹
        out << YAML::Key << "target_traj" << YAML::Value << YAML::BeginSeq;
        for (int j = 0; j < target_traj_.cols(); ++j)
        {
            std::vector<double> traj_step(target_traj_.col(j).data(), target_traj_.col(j).data() + target_traj_.rows());
            out << YAML::Flow << traj_step;
        }
        out << YAML::EndSeq;

        out << YAML::EndMap;

        if (error < 0.0001)
        {
            std::cout << "Target reached at step " << i << ", error norm = " << error << std::endl;
            break;
        }

        Eigen::VectorXd tau = SolveOnceMpc(q, qd);

        pinocchio::aba(model_, data_, q, qd, tau);
        qdd = data_.ddq;
        qd += qdd * delta_t_;
        q += qd * delta_t_;

        std::cout << "Step " << i << ", q: " << q.transpose() << std::endl;
    }

    out << YAML::EndSeq;
    out << YAML::EndMap;

    //使用 ROS 包路径保存
    std::string log_path = ros::package::getPath("robot_mpc") + "/config/mpc_result_log.yaml";
    std::ofstream file(log_path);
    file << out.c_str();
    file.close();

    std::cout << "MPC simulation result saved to " << log_path << std::endl;
}
