#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>
#include "qpOASES.hpp"

class MpcController
{
public:
    MpcController(const std::string& urdf_path, double delta_t, int horizon);

    Eigen::VectorXd SolveOnceMpc(const Eigen::VectorXd& q, const Eigen::VectorXd& qd);

    void RunMpcSimulation(const Eigen::VectorXd& init_state, int total_steps);

private:
    void ComputeDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qd);

    pinocchio::Model model_;
    pinocchio::Data data_;

    int dof_;
    int horizon_;
    double delta_t_;
    int frame_id_;

    Eigen::MatrixXd H_;
    Eigen::VectorXd C_;
    Eigen::MatrixXd J_;

    // QP参数
    Eigen::MatrixXd Q_, W_;
    Eigen::VectorXd q_lower_, q_upper_, qd_lower_, qd_upper_;
};
