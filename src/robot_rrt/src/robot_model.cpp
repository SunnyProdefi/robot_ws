#include "robot_planning/robot_model.h"
#include <iostream>

using namespace pinocchio;

// 定义全局变量
Model model;
Data data(model);
GeometryModel geom_model;
GeometryData geom_data(geom_model);
Eigen::VectorXd q_init_(28);

void addObstacle(pinocchio::GeometryModel &geom_model)
{
    using namespace pinocchio;

    // 创建 Box 类型障碍物（单位: mm）
    GeometryObject obstacle("cube_obstacle", 33, std::make_shared<geometry::Box>(0.25, 0.25, 0.6), SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0.45, 0.3)));
    obstacle.meshColor = Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
    geom_model.addGeometryObject(obstacle);
}

void setupAndSimulateRobot()
{
    const std::string robots_model_path = PINOCCHIO_MODEL_DIR;
    const std::string urdf_filename = robots_model_path + "limbarm_robot.xacro.urdf";
    const std::string srdf_filename = robots_model_path + "limbarm_robot.srdf";

    pinocchio::urdf::buildModel(urdf_filename, model);
    data = Data(model);
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geom_model, robots_model_path);

    // 添加障碍物
    addObstacle(geom_model);

    geom_model.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf_filename);

    geom_data = GeometryData(geom_model);
    pinocchio::srdf::loadReferenceConfigurations(model, srdf_filename);

    // 设置初始配置
    q_init_ = Eigen::VectorXd::Zero(28);
    std::cout << "q_init_: " << q_init_.transpose() << std::endl;
}

bool CollisionCheck(const pinocchio::Model &model, pinocchio::Data &data, const pinocchio::GeometryModel &geom_model, pinocchio::GeometryData &geom_data, const Eigen::VectorXd &q_total, bool verbose)
{
    pinocchio::computeCollisions(model, data, geom_model, geom_data, q_total);
    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {
        const auto &cp = geom_model.collisionPairs[k];
        const auto &result = geom_data.collisionResults[k];
        if (result.isCollision())
        {
            if (verbose)
            {
                std::cout << "Collision detected between pair: " << cp.first << " and " << cp.second << std::endl;
            }
            return true;
        }
    }
    return false;
}

void printGeometryIDs(const pinocchio::GeometryModel &geom_model)
{
    for (const auto &geom_obj : geom_model.geometryObjects)
    {
        std::cout << "Link: " << geom_obj.name << std::endl;
    }
}

void printGeometrySizes(const pinocchio::GeometryModel &geom_model)
{
    for (const auto &geom_obj : geom_model.geometryObjects)
    {
        std::cout << "Geometry Object: " << geom_obj.name << std::endl;
        const auto &shape = geom_obj.geometry;
        std::cout << "Shape type: " << shape->getNodeType() << std::endl;
    }
}
