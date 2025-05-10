#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_truss_link_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 1️⃣ 创建并添加桁架
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "truss_link";

    shapes::Mesh* m = shapes::createMeshFromResource("package://robot_description/meshes/truss_link.STL");
    if (!m)
    {
        ROS_ERROR("Failed to load truss_link.STL mesh!");
        return 1;
    }

    shape_msgs::Mesh mesh_msg;
    shapes::ShapeMsg mesh_msg_tmp;
    shapes::constructMsgFromShape(m, mesh_msg_tmp);
    mesh_msg = boost::get<shape_msgs::Mesh>(mesh_msg_tmp);
    delete m;  // ✅ 释放动态分配的内存

    geometry_msgs::Pose mesh_pose;
    mesh_pose.position.x = 0.6625;
    mesh_pose.position.y = 0.0;
    mesh_pose.position.z = 0.0;

    tf::Quaternion q = tf::createQuaternionFromRPY(1.5708, 0.0, 3.1415);
    tf::quaternionTFToMsg(q, mesh_pose.orientation);

    collision_object.meshes.push_back(mesh_msg);
    collision_object.mesh_poses.push_back(mesh_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface.addCollisionObjects({collision_object});
    ROS_INFO("truss_link added to planning scene.");

    // 1️⃣+ 添加地板作为障碍物
    moveit_msgs::CollisionObject floor_object;
    floor_object.header.frame_id = "world";
    floor_object.id = "floor";

    shape_msgs::SolidPrimitive floor_primitive;
    floor_primitive.type = shape_msgs::SolidPrimitive::BOX;
    floor_primitive.dimensions.resize(3);
    floor_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 5.0;   // 长
    floor_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 5.0;   // 宽
    floor_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.01;  // 高（很薄）

    geometry_msgs::Pose floor_pose;
    floor_pose.position.x = 0.0;
    floor_pose.position.y = 0.0;
    floor_pose.position.z = -0.1;  // 让它贴在z=0平面上
    floor_pose.orientation.w = 1.0;

    floor_object.primitives.push_back(floor_primitive);
    floor_object.primitive_poses.push_back(floor_pose);
    floor_object.operation = floor_object.ADD;

    planning_scene_interface.addCollisionObjects({floor_object});
    ROS_INFO("floor added to planning scene.");

    // 2️⃣ 初始化 PlanningSceneMonitor 并启动监听
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    psm->startSceneMonitor();
    psm->requestPlanningSceneState();

    ros::Duration(1.0).sleep();  // 等待场景同步

    planning_scene::PlanningScenePtr scene = psm->getPlanningScene();
    const auto& world_objects = scene->getWorld()->getObjectIds();

    ROS_INFO("========== Current World Collision Objects ==========");
    for (const auto& obj_id : world_objects)
    {
        ROS_INFO("Object ID: %s", obj_id.c_str());
    }

    return 0;
}
