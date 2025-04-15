// robot_model.h (已修改)
#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#ifndef PINOCCHIO_WITH_HPP_FCL
#define PINOCCHIO_WITH_HPP_FCL
#endif

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

// 最后再加
#include <boost/variant.hpp>

#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/prodefi/robot/robot_ws/src/robot_planning/models/"
#endif

extern pinocchio::Model model;
extern pinocchio::Data data;
extern pinocchio::GeometryModel geom_model;
extern pinocchio::GeometryData geom_data;
extern Eigen::VectorXd q_init_;

void addObstacle(pinocchio::GeometryModel &geom_model);
void setupAndSimulateRobot();
bool CollisionCheck(const pinocchio::Model &model, pinocchio::Data &data, const pinocchio::GeometryModel &geom_model, pinocchio::GeometryData &geom_data, const Eigen::VectorXd &q_total, bool verbose);
void printGeometryIDs(const pinocchio::GeometryModel &geom_model);
void printGeometrySizes(const pinocchio::GeometryModel &geom_model);

#endif  // ROBOT_MODEL_H