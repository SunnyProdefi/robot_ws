#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "force_sine_wrench");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Wrench>("/force_torque_data_3", 10);

    ros::Rate loop_rate(200);
    double t = 0.0;

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
        msg.force.x = -15.0 * sin(2 * M_PI * 0.5 * t);  // 0.5 Hz 正弦扰动
        pub.publish(msg);
        t += 0.01;
        loop_rate.sleep();
    }
}
