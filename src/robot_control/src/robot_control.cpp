#include <ros/ros.h>
#include "motor_driver.h"
#include "Gripper_Driver.h"

extern double q_send[BRANCHN_N][MOTOR_BRANCHN_N];
extern double q_recv[BRANCHN_N][MOTOR_BRANCHN_N];
extern double q_init[BRANCHN_N][MOTOR_BRANCHN_N];

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;

    // 电机初始化

    // 夹爪初始化

    // IMU初始化

    // 六维力初始化

    // control_flag = 0 读取上电状态
    // control_flag = 1 运动到初始状态
    // control_flag = 2 抓取演示
    int control_flag = 0;  // 上电状态

    ros::Rate loop_rate(200);  // 200Hz
    while (ros::ok())
    {
        if (control_flag == 0)
        {
        }
        else if (control_flag == 1)
        {
        }
        else if (control_flag == 2)
        {
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}