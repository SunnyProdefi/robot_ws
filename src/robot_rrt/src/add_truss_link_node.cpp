#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_truss_link_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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
    ROS_INFO("✅ truss_link added to planning scene.");

    ros::waitForShutdown();
    return 0;
}
