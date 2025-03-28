#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <iomanip>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listen_tf_dummy_point3");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Rate rate(10.0);

    // 等待 TF 建立
    listener.waitForTransform("world", "dummy_point3", ros::Time(0), ros::Duration(5.0));
    ROS_INFO("Start listening to transform from 'world' to 'dummy_point3'...");

    while (ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("world", "dummy_point3", ros::Time(0), transform);

            // 平移
            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double z = transform.getOrigin().z();

            // 四元数
            tf::Quaternion q = transform.getRotation();
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            std::cout << std::fixed << std::setprecision(4);
            std::cout << "\n--- TF: world → dummy_point3 ---\n";
            std::cout << "Translation : [x: " << x << ", y: " << y << ", z: " << z << "]\n";
            std::cout << "Quaternion  : [x: " << q.x() << ", y: " << q.y() << ", z: " << q.z() << ", w: " << q.w() << "]\n";
            std::cout << "RPY (deg)   : [roll: " << roll * 180 / M_PI << ", pitch: " << pitch * 180 / M_PI << ", yaw: " << yaw * 180 / M_PI << "]\n";
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
