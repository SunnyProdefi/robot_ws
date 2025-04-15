#ifndef BSPLINEINTERPOLATOR_H
#define BSPLINEINTERPOLATOR_H

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <vector>

class BSplineInterpolator
{
public:
    BSplineInterpolator(int dimension, int degree);

    // 生成插值点，自动计算均匀分布的参数值
    std::vector<Eigen::VectorXd> generatePoints(const std::vector<Eigen::VectorXd> &control_points, int num_samples);

    // 生成插值点，使用 Eigen::MatrixXd 作为控制点输入
    std::vector<Eigen::VectorXd> generatePoints(const Eigen::MatrixXd &control_points, int num_samples);

private:
    int dimension_;
    int degree_;
    typedef Eigen::Spline<double, 12> DynamicSpline;
};

#endif  // BSPLINEINTERPOLATOR_H
