#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

// ✅ 强制启用 FCL 支持（在使用 buildGeom 前必须）
#ifndef PINOCCHIO_WITH_HPP_FCL
#define PINOCCHIO_WITH_HPP_FCL
#endif

// ✅ 基础前向声明
#include <pinocchio/fwd.hpp>

// ✅ Multibody 模型
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

// ✅ 算法部分（配置、几何）
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>

// ✅ URDF / SRDF 解析器（必须）
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

// ✅ HPP-FCL 碰撞几何（注意是 hpp::fcl，不是 coal）
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/mesh_loader/loader.h>  // 加载 mesh 文件用的 MeshLoader（如有）
#include <coal/BVH/BVH_model.h>

#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/distance.hpp"

// ✅ Boost 类型兼容
#include <boost/variant.hpp>

// ✅ 路径定义（建议：未来从 rosparam 获取而非硬编码）
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/prodefi/robot/robot_ws/src/robot_rrt/models/"
#endif

// ✅ 声明外部变量（建议最终封装为类）
extern pinocchio::Model model;
extern pinocchio::Data data;
extern pinocchio::GeometryModel geom_model;
extern pinocchio::GeometryData geom_data;
extern Eigen::VectorXd q_init_;

// ✅ 函数接口（建议最终封装为 RobotModel 类）
void addObstacle(pinocchio::GeometryModel &geom_model);
void setupAndSimulateRobot();
bool CollisionCheck(const pinocchio::Model &model, pinocchio::Data &data, const pinocchio::GeometryModel &geom_model, pinocchio::GeometryData &geom_data, const Eigen::VectorXd &q_total, bool verbose);
void printGeometryIDs(const pinocchio::GeometryModel &geom_model);
void printGeometrySizes(const pinocchio::GeometryModel &geom_model);

#endif  // ROBOT_MODEL_H
