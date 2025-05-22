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
    // static_transformStamped.child_frame_id = "grasp_object_cube";
    static_transformStamped.child_frame_id = "grasp_object";

    // 设置平移部分（例如：x=0.5825, y=0.36, z=-0.07）
    // static_transformStamped.transform.translation.x = 0.5825;
    // static_transformStamped.transform.translation.y = 0.36;
    // static_transformStamped.transform.translation.z = -0.07;

    // // 初始OK
    static_transformStamped.transform.translation.x = 0.59;
    static_transformStamped.transform.translation.y = 0.373;
    static_transformStamped.transform.translation.z = -0.07;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, -0.5238);  // Roll, Pitch, Yaw
    quat.normalize();

    // // 交接OK
    // static_transformStamped.transform.translation.x = 0.7775;
    // static_transformStamped.transform.translation.y = 0.0;
    // static_transformStamped.transform.translation.z = 0.215;
    // tf2::Quaternion quat;
    // quat.setRPY(0, 0, -1.5708);  // Roll, Pitch, Yaw
    // quat.normalize();

    // // 放置OK
    // static_transformStamped.transform.translation.x = 0.927475;
    // static_transformStamped.transform.translation.y = -0.27;
    // static_transformStamped.transform.translation.z = 0.015;
    // tf2::Quaternion quat;
    // quat.setRPY(-1.5708, 0, 0);  // Roll, Pitch, Yaw
    // quat.normalize();

    // cube初始OK
    // static_transformStamped.transform.translation.x = 0.6625;
    // static_transformStamped.transform.translation.y = 0.0;
    // static_transformStamped.transform.translation.z = 0.265;
    // tf2::Quaternion quat;
    // quat.setRPY(0, 0, 0);  // Roll, Pitch, Yaw
    // quat.normalize();

    // // cube放置OK
    // static_transformStamped.transform.translation.x = 0.6925;
    // static_transformStamped.transform.translation.y = -0.1325;
    // static_transformStamped.transform.translation.z = 0.165;
    // tf2::Quaternion quat;
    // quat.setRPY(0, 0, 0);  // Roll, Pitch, Yaw
    // quat.normalize();

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
