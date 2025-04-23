#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_check_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化 PlanningSceneMonitor
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // 初始化后立即启动状态监听
    psm->startStateMonitor();

    // 等待关节状态同步
    ros::Duration(1.0).sleep();

    // 获取当前状态
    robot_state::RobotState& current_state = psm->getPlanningScene()->getCurrentStateNonConst();

    if (!psm->getPlanningScene())
    {
        ROS_ERROR("Unable to get Planning Scene, please check if move_group is started!");
        return 1;
    }

    // 更新当前场景状态
    psm->requestPlanningSceneState();

    // 获取当前机器人状态
    current_state = psm->getPlanningScene()->getCurrentStateNonConst();

    // ✅ 获取当前关节组的关节角（以 arm2 为例）
    const robot_state::JointModelGroup* joint_model_group = current_state.getJointModelGroup("arm2");

    std::vector<double> current_joint_values;
    current_state.copyJointGroupPositions(joint_model_group, current_joint_values);

    // 打印当前关节角
    ROS_INFO("Current arm2 joint angle:");
    for (size_t i = 0; i < current_joint_values.size(); ++i)
    {
        ROS_INFO("Joint %lu: %.4f", i, current_joint_values[i]);
    }

    // 直接用当前状态进行碰撞检测
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.group_name = "arm2";  // 针对 arm2 检测
    collision_request.contacts = true;      // 输出详细接触点
    collision_request.max_contacts = 10;

    psm->getPlanningScene()->checkCollision(collision_request, collision_result, current_state);

    if (collision_result.collision)
    {
        ROS_WARN("The current posture detects a collision!");
        for (const auto& contact : collision_result.contacts)
        {
            ROS_WARN("Collision Pair: %s <--> %s", contact.first.first.c_str(), contact.first.second.c_str());
        }
    }
    else
    {
        ROS_INFO("There is no collision in the current posture!");
    }

    ros::shutdown();
    return 0;
}
