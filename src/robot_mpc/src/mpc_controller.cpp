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

    // 构建状态空间模型 A, B, C
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2 * dof_, 2 * dof_);
    A.block(0, dof_, dof_, dof_) = delta_t_ * Eigen::MatrixXd::Identity(dof_, dof_);

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2 * dof_, dof_);
    B.block(dof_, 0, dof_, dof_) = H_.inverse() * delta_t_;

    Eigen::VectorXd C_state = Eigen::VectorXd::Zero(2 * dof_);
    C_state.tail(dof_) = -H_.inverse() * C_ * delta_t_;

    // 构建 QP (这里略，和你之前逻辑一致)
    // 使用 qpOASES 求解
    // 返回 U.head(dof_)

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(dof_);  // 伪代码
    return tau;
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
