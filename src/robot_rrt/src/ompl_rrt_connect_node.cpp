#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_tools.h>

// OMPL 头文件
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// 全局变量：动态同步的 PlanningSceneMonitor
planning_scene_monitor::PlanningSceneMonitorPtr psm;

// ===== 碰撞检测函数 =====
bool isStateValid(const ob::State* state)
{
    auto scene = psm->getPlanningScene();
    robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();

    const auto* real_state = state->as<ob::RealVectorStateSpace::StateType>();
    std::vector<double> joint_values(7);
    joint_values[0] = 1.5708;

    for (size_t i = 0; i < 6; ++i) joint_values[i + 1] = real_state->values[i];

    robot_state.setJointGroupPositions("arm2", joint_values);

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;

    req.contacts = true;
    req.max_contacts = 1000;
    req.verbose = false;

    // 重点：显式检测附着物体与世界的碰撞
    req.group_name = "";  // 空表示全局检测

    // 获取 ACM，并排除特定碰撞对
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrixNonConst();
    acm.setEntry("Link2_0", "Link_platform", true);

    // 执行检测
    scene->checkCollision(req, res, robot_state, acm);

    if (res.collision)
    {
        ROS_WARN("State in collision! Detected contacts:");
        for (const auto& contact_pair : res.contacts)
        {
            ROS_WARN("Collision: %s <--> %s", contact_pair.first.first.c_str(), contact_pair.first.second.c_str());
        }
        return false;
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ompl_rrt_connect_with_display");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 1️⃣ 初始化 PlanningSceneMonitor（动态同步全局环境）
    psm.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    if (!psm->getPlanningScene())
    {
        ROS_ERROR("Failed to initialize PlanningSceneMonitor!");
        return 1;
    }

    psm->startSceneMonitor();
    ros::Duration(1.0).sleep();  // 等待状态同步

    planning_scene::PlanningScenePtr scene = psm->getPlanningScene();
    robot_state::RobotState& current_state = scene->getCurrentStateNonConst();

    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->requestPlanningSceneState();  // 确保拿到最新的场景和环境信息

    const auto& world_objects = scene->getWorld()->getObjectIds();
    for (const auto& obj_id : world_objects)
    {
        ROS_INFO("World Collision Object: %s", obj_id.c_str());
    }

    // 2️⃣ 添加附着物体 box_object（同步到 RViz 和本地场景）
    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "Link2_6";
    attached_object.object.header.frame_id = "Link2_6";
    attached_object.object.id = "box_object";

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions = {0.2, 0.03, 0.03};

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.05;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.17;

    attached_object.object.primitives.push_back(box);
    attached_object.object.primitive_poses.push_back(box_pose);
    attached_object.object.operation = attached_object.object.ADD;

    // 明确设置 touch_links，只允许与自身接触
    attached_object.touch_links = {"Link2_6"};

    psi.applyAttachedCollisionObject(attached_object);
    psm->getPlanningScene()->processAttachedCollisionObjectMsg(attached_object);

    ROS_INFO("Attached object added. Waiting for RViz sync...");
    ros::Duration(2.0).sleep();

    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->requestPlanningSceneState();  // 确保拿到最新的场景和环境信息

    const auto& world_objects_new = scene->getWorld()->getObjectIds();
    for (const auto& obj_id : world_objects_new)
    {
        ROS_INFO("World Collision Object: %s", obj_id.c_str());
    }

    // 使用默认 ACM，不对碰撞矩阵做修改
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.contacts = true;      // 获取详细接触点信息
    req.max_contacts = 1000;  // 设置最大接触数量，可根据需求调整

    scene->checkCollision(req, res, current_state);

    if (res.collision)
    {
        ROS_WARN("Detected collisions in the current scene:");

        for (const auto& contact_pair : res.contacts)
        {
            const std::string& link1 = contact_pair.first.first;
            const std::string& link2 = contact_pair.first.second;

            ROS_WARN("Collision: %s <--> %s", link1.c_str(), link2.c_str());
        }
    }
    else
    {
        ROS_INFO("No collision detected in the current scene.");
    }

    // 输出当前环境信息
    for (const auto& obj_id : psm->getPlanningScene()->getWorld()->getObjectIds()) ROS_INFO("World Object: %s", obj_id.c_str());

    std::vector<const moveit::core::AttachedBody*> attached_bodies;
    psm->getPlanningScene()->getCurrentState().getAttachedBodies(attached_bodies);
    for (const auto& body : attached_bodies) ROS_INFO("Attached Object: %s", body->getName().c_str());

    // 3️⃣ 定义 OMPL 状态空间
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(6));
    ob::RealVectorBounds bounds(6);
    bounds.setLow(0, -2.3562);
    bounds.setHigh(0, 4.7124);
    bounds.setLow(1, -2.0944);
    bounds.setHigh(1, 1.5708);
    bounds.setLow(2, -2.5307);
    bounds.setHigh(2, 2.5307);
    bounds.setLow(3, -6.28);
    bounds.setHigh(3, 6.28);
    bounds.setLow(4, -1.9199);
    bounds.setHigh(4, 1.9199);
    bounds.setLow(5, -6.28);
    bounds.setHigh(5, 6.28);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(isStateValid);

    // 4️⃣ 设置起点和终点
    ob::ScopedState<> start(space), goal(space);
    start = {-1.35192, -1.448481, -2.243493, 3.199319, 0.03093018, -1.050275};
    goal = {-1.35399, -1.63415, -2.29649, 3.1416, 0.1636391, -0.9914737};

    if (!isStateValid(start.get()))
    {
        ROS_ERROR("Start state is invalid due to collision!");
        return 1;
    }

    ROS_INFO("Start state valid. Begin planning...");

    ss.setStartAndGoalStates(start, goal);
    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
    ss.setup();

    // 5️⃣ 执行规划
    if (ss.solve(5.0))
    {
        ROS_INFO("Planning succeeded!");
        ss.simplifySolution();
        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
    {
        ROS_WARN("Planning failed. No solution found.");
    }

    ros::waitForShutdown();
    return 0;
}
