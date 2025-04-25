#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_tools.h>
#include <robot_rrt/CheckCollision.h>  // 替换为你的包名
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>

planning_scene_monitor::PlanningSceneMonitorPtr psm;
robot_model::RobotModelPtr kinematic_model;
collision_detection::AllowedCollisionMatrix global_acm;

enum JointID : int
{
    // 先放平台关节
    JOINT_PLATFORM = 0,

    // Branch 1
    JOINT1_0,
    JOINT1_1,
    JOINT1_2,
    JOINT1_3,
    JOINT1_4,
    JOINT1_5,
    JOINT1_6,
    LINK1_FINGER,

    // Branch 2
    JOINT2_0,
    JOINT2_1,
    JOINT2_2,
    JOINT2_3,
    JOINT2_4,
    JOINT2_5,
    JOINT2_6,
    LINK2_FINGER,

    // Branch 3
    JOINT3_0,
    JOINT3_1,
    JOINT3_2,
    JOINT3_3,
    JOINT3_4,
    JOINT3_5,
    JOINT3_6,
    LINK3_FINGER,

    // Branch 4
    JOINT4_0,
    JOINT4_1,
    JOINT4_2,
    JOINT4_3,
    JOINT4_4,
    JOINT4_5,
    JOINT4_6,
    LINK4_FINGER,

    JOINT1_PLATLINK,
    JOINT2_PLATLINK,
    JOINT3_PLATLINK,
    JOINT4_PLATLINK,

    // 用来标记总数量，必须放在最后
    JOINT_COUNT
};

// 建立一个数组，把上面枚举的每个 ID 与具体关节名称一一对应
static const std::array<std::string, JOINT_COUNT> JOINT_NAMES = {
    /* JOINT_PLATFORM */ "Joint_platform",

    /* JOINT1_0 */ "Joint1_0",
    /* JOINT1_1 */ "Joint1_1",
    /* JOINT1_2 */ "Joint1_2",
    /* JOINT1_3 */ "Joint1_3",
    /* JOINT1_4 */ "Joint1_4",
    /* JOINT1_5 */ "Joint1_5",
    /* JOINT1_6 */ "Joint1_6",
    /* LINK1_FINGER */ "Link1_finger_joint",

    /* JOINT2_0 */ "Joint2_0",
    /* JOINT2_1 */ "Joint2_1",
    /* JOINT2_2 */ "Joint2_2",
    /* JOINT2_3 */ "Joint2_3",
    /* JOINT2_4 */ "Joint2_4",
    /* JOINT2_5 */ "Joint2_5",
    /* JOINT2_6 */ "Joint2_6",
    /* LINK2_FINGER */ "Link2_finger_joint",

    /* JOINT3_0 */ "Joint3_0",
    /* JOINT3_1 */ "Joint3_1",
    /* JOINT3_2 */ "Joint3_2",
    /* JOINT3_3 */ "Joint3_3",
    /* JOINT3_4 */ "Joint3_4",
    /* JOINT3_5 */ "Joint3_5",
    /* JOINT3_6 */ "Joint3_6",
    /* LINK3_FINGER */ "Link3_finger_joint",

    /* JOINT4_0 */ "Joint4_0",
    /* JOINT4_1 */ "Joint4_1",
    /* JOINT4_2 */ "Joint4_2",
    /* JOINT4_3 */ "Joint4_3",
    /* JOINT4_4 */ "Joint4_4",
    /* JOINT4_5 */ "Joint4_5",
    /* JOINT4_6 */ "Joint4_6",
    /* LINK4_FINGER */ "Link4_finger_joint",

    /* JOINT1_PLATLINK */ "Joint1_platlink",
    /* JOINT2_PLATLINK */ "Joint2_platlink",
    /* JOINT3_PLATLINK */ "Joint3_platlink",
    /* JOINT4_PLATLINK */ "Joint4_platlink",
};

// 使用全局变量 kinematic_model
std::vector<std::string> getLinksInGroup(const std::string& group_name)
{
    std::vector<std::string> links;
    const robot_model::JointModelGroup* joint_group = kinematic_model->getJointModelGroup(group_name);
    if (joint_group)
    {
        links = joint_group->getLinkModelNames();
        ROS_INFO("Group [%s] has %lu links.", group_name.c_str(), links.size());
    }
    else
    {
        ROS_WARN("Group [%s] not found!", group_name.c_str());
    }
    return links;
}

void ignoreGroupCollisions(collision_detection::AllowedCollisionMatrix& acm, const std::vector<std::string>& groupA, const std::vector<std::string>& groupB)
{
    for (const auto& linkA : groupA)
    {
        for (const auto& linkB : groupB)
        {
            acm.setEntry(linkA, linkB, true);
        }
    }
}

bool checkCollisionCallback(robot_rrt::CheckCollision::Request& req, robot_rrt::CheckCollision::Response& res)
{
    planning_scene::PlanningScenePtr scene = psm->getPlanningScene();
    robot_state::RobotState& current_state = scene->getCurrentStateNonConst();

    const std::vector<std::string>& moveit_joint_names = current_state.getVariableNames();

    if (req.joint_positions.size() != JOINT_COUNT)
    {
        ROS_ERROR("Joint positions size (%lu) does not match expected size (%d).", req.joint_positions.size(), JOINT_COUNT);
        res.collision = true;
        return true;
    }

    // 构建一个 map，便于查找你的关节名对应的角度值
    std::map<std::string, double> joint_position_map;
    for (size_t i = 0; i < JOINT_COUNT; ++i)
    {
        joint_position_map[JOINT_NAMES[i]] = req.joint_positions[i];
    }

    // 只更新请求中包含的关节，其余关节保持当前状态
    for (const auto& pair : joint_position_map)
    {
        current_state.setVariablePosition(pair.first, pair.second);
    }

    // 其余未指定的关节角度，不做任何修改（保持 current_state 的原始值）

    // 执行碰撞检测
    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult cres;
    creq.contacts = true;
    creq.max_contacts = 1000;

    scene->checkCollision(creq, cres, current_state, global_acm);

    res.collision = cres.collision;

    if (cres.collision)
    {
        for (const auto& contact_pair : cres.contacts)
        {
            std::string collision_info = contact_pair.first.first + " <--> " + contact_pair.first.second;
            res.collision_pairs.push_back(collision_info);
        }
        ROS_WARN("Collision detected!");
    }
    else
    {
        ROS_INFO("No collision detected.");
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_collision_server");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化 PlanningSceneMonitor
    psm.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    if (!psm->getPlanningScene())
    {
        ROS_ERROR("Unable to get Planning Scene, please check if move_group is started!");
        return 1;
    }

    psm->startStateMonitor();
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->requestPlanningSceneState();

    // 初始化 RobotModel
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model = robot_model_loader.getModel();

    // ===== 初始化全局 ACM =====
    global_acm = psm->getPlanningScene()->getAllowedCollisionMatrix();

    auto arm1_with_hand1 = getLinksInGroup("arm1_with_hand1");
    auto arm2_with_hand2 = getLinksInGroup("arm2_with_hand2");
    auto arm3_with_hand3 = getLinksInGroup("arm3_with_hand3");
    auto arm4_with_hand4 = getLinksInGroup("arm4_with_hand4");
    auto platform_links = getLinksInGroup("platform");

    ignoreGroupCollisions(global_acm, arm2_with_hand2, arm1_with_hand1);
    ignoreGroupCollisions(global_acm, arm2_with_hand2, arm4_with_hand4);
    ignoreGroupCollisions(global_acm, arm2_with_hand2, platform_links);

    ignoreGroupCollisions(global_acm, arm3_with_hand3, arm1_with_hand1);
    ignoreGroupCollisions(global_acm, arm3_with_hand3, arm4_with_hand4);
    ignoreGroupCollisions(global_acm, arm3_with_hand3, platform_links);

    ROS_INFO("Global Allowed Collision Matrix initialized.");

    // 广播服务
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("check_collision", checkCollisionCallback);
    ROS_INFO("Check Collision Service is ready.");

    ros::waitForShutdown();
    return 0;
}
