#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "full_collision_check_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    if (!psm->getPlanningScene())
    {
        ROS_ERROR("Unable to get Planning Scene, please check if move_group is started!");
        return 1;
    }

    psm->startStateMonitor();
    ros::Duration(1.0).sleep();  // 等待状态同步

    planning_scene::PlanningScenePtr scene = psm->getPlanningScene();
    robot_state::RobotState& current_state = scene->getCurrentStateNonConst();
    // 获取关节组名称（可选，针对特定的 group）
    const std::vector<std::string>& joint_names = current_state.getVariableNames();

    // 遍历并打印每个关节的当前值
    for (const auto& joint_name : joint_names)
    {
        double joint_value = current_state.getVariablePosition(joint_name);
        ROS_INFO("Joint: %s  |  Position: %.6f", joint_name.c_str(), joint_value);
    }

    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->requestPlanningSceneState();  // 确保拿到最新的场景和环境信息

    const auto& world_objects = scene->getWorld()->getObjectIds();
    for (const auto& obj_id : world_objects)
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

    ros::shutdown();
    return 0;
}
