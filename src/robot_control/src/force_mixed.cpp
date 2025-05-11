#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "torque_const_wrench");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Wrench>("/force_torque_data_3", 10);

    ros::Rate loop_rate(200);

    ros::Time start_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Duration elapsed = ros::Time::now() - start_time;
        if (elapsed.toSec() > 5.0)
        {
            ROS_INFO("5 seconds elapsed. Stopping force publication.");
            break;  // 停止发布
        }

        geometry_msgs::Wrench msg;
        msg.torque.z = -10.0;  // 恒定扭矩
        pub.publish(msg);
        loop_rate.sleep();
    }
}
