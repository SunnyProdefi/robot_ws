#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>

// OMPL 头文件
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

planning_scene::PlanningScenePtr planning_scene_ptr;

bool isStateValid(const ob::State *state)
{
    robot_state::RobotState robot_state = planning_scene_ptr->getCurrentStateNonConst();

    const ob::RealVectorStateSpace::StateType *real_state = state->as<ob::RealVectorStateSpace::StateType>();
    std::vector<double> joint_values(7);
    joint_values[0] = 1.5708;  // 固定第一个关节

    for (size_t i = 0; i < 6; ++i) joint_values[i + 1] = real_state->values[i];

    robot_state.setJointGroupPositions("arm2", joint_values);

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;

    planning_scene_ptr->checkCollision(req, res, robot_state);

    return !res.collision;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_rrt_connect_with_display");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 1️⃣ 初始化 MoveIt 的 PlanningSceneInterface（用于RViz显示）
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 2️⃣ 加载机器人模型，创建本地 PlanningScene（用于碰撞检测）
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene_ptr = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

    // 3️⃣ 添加附着物体（通过 MoveIt 发布到 RViz）
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "Link2_6";
    attached_object.object.header.frame_id = "Link2_6";
    attached_object.object.id = "box_object";

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions = {0.2, 0.03, 0.03};

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.1;

    attached_object.object.primitives.push_back(box);
    attached_object.object.primitive_poses.push_back(box_pose);
    attached_object.object.operation = attached_object.object.ADD;

    planning_scene_interface.applyAttachedCollisionObject(attached_object);
    planning_scene_ptr->processAttachedCollisionObjectMsg(attached_object);

    ROS_INFO("Attached object added. Waiting for RViz to sync...");
    ros::Duration(2.0).sleep();  // 等待 RViz 同步显示

    // 4️⃣ 定义 OMPL 状态空间（6维，规划后6个关节）
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(6));
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-3.14);
    bounds.setHigh(3.14);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(isStateValid);

    // 5️⃣ 设置起点和终点
    ob::ScopedState<> start(space);
    start = {-1.35192, -1.448481, -2.243493, 3.199319, 0.03093018, -1.050275};

    ob::ScopedState<> goal(space);
    goal = {-1.35399, -1.63415, -2.29649, 3.1416, 0.1636391, -0.9914737};

    ss.setStartAndGoalStates(start, goal);
    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
    ss.setup();

    // 6️⃣ 执行规划
    ob::PlannerStatus solved = ss.solve(5.0);

    if (solved)
    {
        ROS_INFO("Found solution!");
        ss.simplifySolution();
        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
    {
        ROS_WARN("No solution found.");
    }

    ros::waitForShutdown();
    return 0;
}
