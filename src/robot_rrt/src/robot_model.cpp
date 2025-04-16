#include "robot_rrt/robot_model.h"
#include <iostream>

using namespace pinocchio;
// using namespace pinocchio::fcl;

// 定义全局变量
Model model;
Data data(model);
GeometryModel geom_model;
GeometryData geom_data(geom_model);
Eigen::VectorXd q_init_(28);

void addObstacle(pinocchio::GeometryModel &geom_model)
{
    // 创建障碍物的几何形状，立方体大小调整为 0.25 x 0.25 x 0.65
    auto cube_shape = std::make_shared<hpp::fcl::Box>(250, 250, 600);

    // 定义障碍物的位置和方向
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();  // 无旋转
    Eigen::Vector3d trans(0, 450, 300);
    pinocchio::SE3 placement(rot, trans);

    // 创建并添加 GeometryObject
    GeometryObject obstacle("cube_obstacle", 33, cube_shape, placement);
    obstacle.meshColor = Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);  // 颜色调整为纯红色, 透明度为1
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
    pinocchio::computeCollisions(model, data, geom_model, geom_data, q_total, true);
    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {
        const pinocchio::CollisionPair &cp = geom_model.collisionPairs[k];
        const hpp::fcl::CollisionResult &cr = geom_data.collisionResults[k];
        if (cr.isCollision())
        {
            if (verbose)
            {
                std::cout << "Collision detected between pair: " << cp.first << " and " << cp.second << std::endl;
            }
            return true;  // Return immediately if a collision is detected
        }
        else
        {
            if (verbose)
            {
                // std::cout << "No collision detected between pair: " << cp.first
                //           << " and " << cp.second << std::endl;
            }
        }
    }
    return false;  // Return false if no collisions are detected
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
