// ompl_rrt_service_node.cpp

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_tools.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>
#include <robot_rrt/RRTPlanPath.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

planning_scene_monitor::PlanningSceneMonitorPtr psm;

bool isStateValid(const ob::State* state)
{
    // psm->requestPlanningSceneState();
    auto scene = psm->getPlanningScene();
    robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();

    const auto* real_state = state->as<ob::RealVectorStateSpace::StateType>();
    std::vector<double> joint_values(7);
    joint_values[0] = 1.5708;
    for (size_t i = 0; i < 6; ++i) joint_values[i + 1] = real_state->values[i];

    robot_state.setJointGroupPositions("arm2", joint_values);
    robot_state.update();  // 必须更新

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.contacts = true;
    req.max_contacts = 1000;
    req.verbose = false;

    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrixNonConst();
    acm.setEntry("Link2_0", "Link_platform", true);

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

void attachBoxObject()
{
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
    attached_object.touch_links = {"Link2_6"};

    psi.applyAttachedCollisionObject(attached_object);

    if (psm && psm->getPlanningScene())
    {
        psm->getPlanningScene()->processAttachedCollisionObjectMsg(attached_object);
        ROS_INFO("Attached object 'box_object' added.");
    }
}

bool planCallback(robot_rrt::RRTPlanPath::Request& req, robot_rrt::RRTPlanPath::Response& res)
{
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
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);

    auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
    planner->setRange(0.2);
    ss.setPlanner(planner);

    ob::ScopedState<> start(space), goal(space);
    for (int i = 0; i < 6; ++i)
    {
        start[i] = req.start[i];
        goal[i] = req.goal[i];
    }

    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->requestPlanningSceneState();  // 确保拿到最新的场景和环境信息

    if (!isStateValid(start.get()))
    {
        res.success = false;
        res.message = "Start state is in collision.";
        return true;
    }

    ss.setStartAndGoalStates(start, goal);
    ss.setup();

    if (ss.solve(5000.0))
    {
        ROS_INFO("Planning succeeded!");
        // ss.simplifySolution();
        ss.getSolutionPath().printAsMatrix(std::cout);
        const auto& path = ss.getSolutionPath().getStates();

        std::string pkg_path = ros::package::getPath("robot_rrt");
        std::string yaml_path = pkg_path + "/config/planned_path.yaml";

        YAML::Emitter out;
        out << YAML::BeginSeq;
        for (const auto* state : path)
        {
            auto* s = state->as<ob::RealVectorStateSpace::StateType>();
            std::vector<double> vec(s->values, s->values + 6);
            out << YAML::Flow << YAML::Value << vec;  // ✅ 一行一组数
        }
        out << YAML::EndSeq;

        std::ofstream fout(yaml_path);
        fout << out.c_str();
        fout.close();

        res.success = true;
        res.message = "Path planned and saved to " + yaml_path;
    }
    else
    {
        res.success = false;
        res.message = "Planning failed.";
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ompl_rrt_service_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    psm.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    if (!psm->getPlanningScene())
    {
        ROS_ERROR("Failed to initialize PlanningSceneMonitor!");
        return 1;
    }

    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->requestPlanningSceneState();
    ros::Duration(1.0).sleep();  // 等待同步完成

    attachBoxObject();  // 添加附着物体！

    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->requestPlanningSceneState();  // 确保拿到最新的场景和环境信息

    ros::ServiceServer service = nh.advertiseService("rrt_plan_path", planCallback);
    ROS_INFO("OMPL RRTConnect planner service with attached object ready.");
    ros::waitForShutdown();
    return 0;
}
