#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <random>

#include "robot_rrt/BSplineInterpolator.h"
#include "robot_rrt/robot_model.h"
namespace ob = ompl::base;
namespace og = ompl::geometric;

// Global variables from your robot model
extern pinocchio::Model model;
extern pinocchio::Data data;
extern pinocchio::GeometryModel geom_model;
extern pinocchio::GeometryData geom_data;
Eigen::MatrixXd pathPoints;  // 全局矩阵，未初始化大小
// Modified state validity checker to include collision detection
bool isStateValid(const ob::State *state)
{
    const auto *realState = state->as<ob::RealVectorStateSpace::StateType>();
    Eigen::VectorXd q_total = q_init_;  // 初始化为初始位姿

    // 填充第15~20和22~27自由度
    for (size_t i = 15; i < 21; ++i)
    {
        q_total[i] = realState->values[i - 15];
    }
    for (size_t i = 22; i < 28; ++i)
    {
        q_total[i] = realState->values[i - 16];  // 保证 offset 正确
    }

    // 使用你自己的碰撞检测函数
    bool hasCollision = CollisionCheck(model, data, geom_model, geom_data, q_total, true);

    if (hasCollision)
    {
        std::cout << "Collision detected" << std::endl;
        return false;
    }
    else
    {
        std::cout << "No collision detected" << std::endl;
        return true;
    }
}

void planWithRRT(const Eigen::VectorXd &q_init, const Eigen::VectorXd &q_goal, const Eigen::VectorXd &lower_limits, const Eigen::VectorXd &upper_limits)
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(12);  // Use the correct number of DOF

    ob::RealVectorBounds bounds(12);
    for (size_t i = 0; i < 12; ++i)
    {
        bounds.setLow(i, lower_limits[i]);
        bounds.setHigh(i, upper_limits[i]);
    }
    space->setBounds(bounds);
    // 打印边界信息
    std::cout << "State space bounds set to:" << std::endl;
    for (size_t i = 0; i < 12; ++i)
    {
        std::cout << "Dimension " << i + 1 << ": Low = " << bounds.low[i] << ", High = " << bounds.high[i] << std::endl;
    }
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(isStateValid);

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    // 使用q_init作为初始状态
    for (size_t i = 0; i < 12; ++i)
    {
        start[i] = q_init[i];
    }
    // 使用q_goal作为目标状态
    for (size_t i = 0; i < 12; ++i)
    {
        goal[i] = q_goal[i];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::RRTConnect>(si);
    planner->setProblemDefinition(pdef);
    planner->setRange(0.1);  // 将步长设置为0.1
    planner->setup();

    ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ob::PathPtr path = pdef->getSolutionPath();
        auto geoPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(path);
        if (geoPath)
        {
            // 获取路径中的点数
            std::size_t pathLength = geoPath->getStateCount();
            // 假设每个状态的维度已知并且为 `stateDimension`
            std::size_t stateDimension = geoPath->getSpaceInformation()->getStateSpace()->as<ob::RealVectorStateSpace>()->getDimension();
            std::cout << "Path length: " << pathLength << std::endl;
            std::cout << "State dimension: " << stateDimension << std::endl;
            // 初始化全局矩阵的大小
            pathPoints.resize(stateDimension, pathLength);

            // 遍历路径中的每个状态，将其转换为Eigen向量
            for (std::size_t i = 0; i < pathLength; ++i)
            {
                const auto *state = geoPath->getState(i)->as<ob::RealVectorStateSpace::StateType>();
                // 将每个状态的数据复制到Eigen矩阵中的相应列
                for (std::size_t j = 0; j < stateDimension; ++j)
                {
                    pathPoints(j, i) = state->values[j];
                }
            }
        }
        else
        {
            std::cout << "Path could not be converted to PathGeometric." << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

// 设置q_goal，使其在关节限制内随机
void setQGoal(Eigen::VectorXd &q_goal, const Eigen::VectorXd &lower_limits, const Eigen::VectorXd &upper_limits)
{
    std::random_device rd;   // 随机数生成器设备
    std::mt19937 gen(rd());  // 标准 mersenne_twister_engine
    q_goal.resize(12);       // 确保q_goal有正确的大小

    for (size_t i = 0; i < 12; ++i)
    {
        std::uniform_real_distribution<> dis(lower_limits[i], upper_limits[i]);
        q_goal[i] = dis(gen);  // 生成一个在给定范围内的随机数
    }
}

// 转换角度从度到弧度
inline double degreesToRadians(double degrees) { return degrees * M_PI / 180.0; }

// 设置q_goal，检查角度是否在限制范围内
void setQGoal(Eigen::VectorXd &q_goal, const Eigen::VectorXd &angles_degrees, const Eigen::VectorXd &lower_limits, const Eigen::VectorXd &upper_limits)
{
    q_goal.resize(12);  // 确保q_goal有正确的大小
    for (size_t i = 0; i < 12; ++i)
    {
        double rad_angle = degreesToRadians(angles_degrees[i]);

        if (rad_angle < lower_limits[i] || rad_angle > upper_limits[i])
        {
            std::cout << "Error: Angle for joint " << i << " is out of bounds: " << angles_degrees[i] << " degrees (Limits: " << lower_limits[i] << " to " << upper_limits[i] << " degrees)." << std::endl;
        }
        else
        {
            q_goal[i] = rad_angle;  // 直接赋值已转换的弧度
        }
    }
}

void saveMatrixToYAML(const Eigen::MatrixXd &matrix, const std::string &filename)
{
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (int i = 0; i < matrix.cols(); ++i)
    {
        out << YAML::Flow << YAML::BeginSeq;
        for (int j = 0; j < matrix.rows(); ++j)
        {
            out << matrix(j, i);  // 注意是列为状态
        }
        out << YAML::EndSeq;
    }
    out << YAML::EndSeq;

    std::ofstream fout(filename);
    if (!fout.is_open())
    {
        ROS_ERROR_STREAM("Failed to open file: " << filename);
        return;
    }
    fout << out.c_str();
    fout.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_rrt_node");
    ros::NodeHandle nh;
    ROS_INFO("robot_rrt_node started.");

    setupAndSimulateRobot();

    printGeometryIDs(geom_model);
    printGeometrySizes(geom_model);

    std::cout << "Robot model initialized" << std::endl;
    std::cout << "Robot model has " << model.nq << " DOF" << std::endl;
    // 关节角度上下限
    std::cout << "Joint limits: " << model.lowerPositionLimit.transpose() << std::endl << model.upperPositionLimit.transpose() << std::endl;
    // 提取并显示特定关节的位置限制
    Eigen::VectorXd lower_limits(12);  // 存储所需关节的下限
    Eigen::VectorXd upper_limits(12);  // 存储所需关节的上限

    // 提取第14到20个关节的位置限制
    lower_limits.head(6) = model.lowerPositionLimit.segment(15, 6);
    upper_limits.head(6) = model.upperPositionLimit.segment(15, 6);

    // 提取第22到27个关节的位置限制
    lower_limits.tail(6) = model.lowerPositionLimit.tail(6);
    upper_limits.tail(6) = model.upperPositionLimit.tail(6);

    std::cout << "Selected joint lower limits: " << lower_limits.transpose() << std::endl;
    std::cout << "Selected joint upper limits: " << upper_limits.transpose() << std::endl;

    // 初始化q_init和q_goal
    Eigen::VectorXd q_init(12), q_goal(12);
    q_init << q_init_.segment(15, 6), q_init_.tail(6);
    std::cout << "q_init: " << q_init.transpose() << std::endl;

    // 设置q_goal（随机角度）
    // setQGoal(q_goal, lower_limits, upper_limits);
    // std::cout << "q_goal: " << q_goal.transpose() << std::endl;

    // 设置q_goal（指定角度）
    Eigen::VectorXd angles_degrees(12);
    angles_degrees << 40, 6, 93, 33, 100, -40, -27, 13, 43, -21, 63, 57;
    // angles_degrees << -10, -50, 110, -30, 55, -10, 10, -50, 110, 30, 55, 10;
    setQGoal(q_goal, angles_degrees, lower_limits, upper_limits);
    std::cout << "q_goal: " << q_goal.transpose() << std::endl;

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    planWithRRT(q_init, q_goal, lower_limits, upper_limits);

    // 保存路径点
    saveMatrixToYAML(pathPoints, "pathPoints.yaml");
    ROS_INFO("Path points saved to pathPoints.yaml");

    // 插值
    BSplineInterpolator interpolator(pathPoints.rows(), 5);
    std::vector<Eigen::VectorXd> interpolated_points = interpolator.generatePoints(pathPoints, pathPoints.cols() * 5);

    // 转换插值点为矩阵
    Eigen::MatrixXd interpMat(pathPoints.rows(), interpolated_points.size());
    for (size_t i = 0; i < interpolated_points.size(); ++i)
    {
        interpMat.col(i) = interpolated_points[i];
    }

    saveMatrixToYAML(interpMat, "interpolatedPoints.yaml");
    ROS_INFO("Interpolated points saved to interpolatedPoints.yaml");

    ros::spinOnce();  // 可选，如果后续要扩展订阅者

    return 0;
}