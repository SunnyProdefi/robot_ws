#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_pose_static_tf_publisher");

    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    // 设置时间戳和坐标系
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "grasp_object";

    // 设置平移部分（例如：x=0.5825, y=0.36, z=-0.07）
    // static_transformStamped.transform.translation.x = 0.5825;
    // static_transformStamped.transform.translation.y = 0.36;
    // static_transformStamped.transform.translation.z = -0.07;

    static_transformStamped.transform.translation.x = 0.5825;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = -0.07;

    // 设置旋转部分（例如绕Z轴 -30° = -0.5238 rad）
    tf2::Quaternion quat;
    quat.setRPY(0, 0, -0.5238);  // Roll, Pitch, Yaw
    quat.normalize();

    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();

    // 发布静态 TF（只需发布一次即可）
    static_broadcaster.sendTransform(static_transformStamped);

    ROS_INFO("Static transform from 'world' to 'grasp_object' published.");
    ros::spin();

    return 0;
}
