#include "robot_planning/BSplineInterpolator.h"
#include <Eigen/Dense>
#include <iostream>

BSplineInterpolator::BSplineInterpolator(int dimension, int degree) : dimension_(dimension), degree_(degree) {}

std::vector<Eigen::VectorXd> BSplineInterpolator::generatePoints(const std::vector<Eigen::VectorXd> &control_points, int num_samples)
{
    Eigen::MatrixXd points(dimension_, control_points.size());
    for (size_t i = 0; i < control_points.size(); ++i)
    {
        points.col(i) = control_points[i];
    }

    auto spline = Eigen::SplineFitting<DynamicSpline>::Interpolate(points, degree_);

    std::vector<Eigen::VectorXd> spline_points;
    double t_step = 1.0 / (num_samples - 1);
    for (int i = 0; i < num_samples; ++i)
    {
        double t = i * t_step;
        spline_points.push_back(spline(t));
    }

    return spline_points;
}

std::vector<Eigen::VectorXd> BSplineInterpolator::generatePoints(const Eigen::MatrixXd &control_points, int num_samples)
{
    auto spline = Eigen::SplineFitting<DynamicSpline>::Interpolate(control_points, degree_);

    std::vector<Eigen::VectorXd> spline_points;
    double t_step = 1.0 / (num_samples - 1);
    for (int i = 0; i < num_samples; ++i)
    {
        double t = i * t_step;
        spline_points.push_back(spline(t));
    }

    return spline_points;
}
