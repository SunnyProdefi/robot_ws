#include <ros/ros.h>
#include "motor_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;

    // Motor initialization
    if (Inital_Motor_Connect() == true)
    {
        cout << "Motor initialization successful" << endl;
    }
    else
    {
        cout << "Motor initialization failed" << endl;
        return 0;
    }

    // Enable all motors
    Set_Motor_ALL_Enable(1);
    if (isEnable == true)
    {
        cout << "Motors enabled successfully" << endl;
    }
    else
    {
        cout << "Motor enabling failed" << endl;
        return 0;
    }

    // Set motor mode (position mode)
    Motor_Set_Func_ALL(MOTORCOMMAND_POSITION);
    cout << "Motor configuration completed" << endl;

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