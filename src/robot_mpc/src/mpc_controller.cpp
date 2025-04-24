#include "robot_mpc/mpc_controller.h"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/aba.hpp>  // aba

#include <iostream>

MpcController::MpcController(const std::string& urdf_path, double delta_t, int horizon) : delta_t_(delta_t), horizon_(horizon)
{
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    dof_ = model_.nv;

    frame_id_ = model_.getFrameId("ee_link");  // 假设末端执行器
}

void MpcController::ComputeDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qd)
{
    pinocchio::crba(model_, data_, q);
    H_ = data_.M;

    pinocchio::rnea(model_, data_, q, qd, Eigen::VectorXd::Zero(dof_));
    C_ = data_.tau;

    pinocchio::computeFrameJacobian(model_, data_, q, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_);
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
    Eigen::VectorXd R = Eigen::VectorXd::Zero(2 * dof_ * p);
    for (int i = 0; i < p; ++i)
    {
        R.segment(i * 2 * dof_, 2 * dof_) = target_state_;  // target_state_ 需为 2*dof_ 维
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

    return U_opt.head(dof_);  // 返回当前时刻 τ
}

void MpcController::RunMpcSimulation(const Eigen::VectorXd& init_state, int total_steps)
{
    Eigen::VectorXd q = init_state.head(dof_);
    Eigen::VectorXd qd = init_state.tail(dof_);
    Eigen::VectorXd qdd = Eigen::VectorXd::Zero(dof_);

    for (int i = 0; i < total_steps; ++i)
    {
        Eigen::VectorXd tau = SolveOnceMpc(q, qd);
        pinocchio::aba(model_, data_, q, qd, tau);
        qdd = data_.ddq;

        qd += qdd * delta_t_;
        q += qd * delta_t_;
        std::cout << "Step " << i << ", q: " << q.transpose() << std::endl;
    }
}
