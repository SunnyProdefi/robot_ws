#ifndef ROBOT_PLANNING_IK_SOLVER_H
#define ROBOT_PLANNING_IK_SOLVER_H

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace robot_planning {

struct IKSolverConfig {
    std::string urdf_path;
    std::string joint_limits_path;
    std::string init_config_path;
    double eps = 1e-3;        // 收敛阈值
    int max_iterations = 50000; // 最大迭代次数
    double dt = 6e-1;         // 更新步长
    double damp = 1e-2;       // 阻尼因子
};

struct IKSolverResult {
    bool success;
    std::vector<double> joint_angles;
    std::string message;
};

class IKSolver {
public:
    IKSolver(const IKSolverConfig& config);
    
    // 求解逆运动学
    IKSolverResult solveIK(const Eigen::Vector3d& target_pos_R,
                          const Eigen::Matrix3d& target_ori_R,
                          const Eigen::Vector3d& target_pos_L,
                          const Eigen::Matrix3d& target_ori_L);

    // 保存结果到YAML文件
    void saveResult(const std::string& filename, const std::vector<double>& qIk);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    IKSolverConfig config_;
    Eigen::VectorXd angle_min_;
    Eigen::VectorXd angle_max_;
    Eigen::VectorXd floating_min_;
    Eigen::VectorXd floating_max_;
};

} // namespace robot_planning

#endif // ROBOT_PLANNING_IK_SOLVER_H 