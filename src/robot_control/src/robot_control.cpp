#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include "motor_driver.h"

// IMU data storage
sensor_msgs::Imu imu_data;
bool IMU_connect_flag = false;

// Force-torque data storage for 4 sensors
geometry_msgs::Wrench force_data_1;
geometry_msgs::Wrench force_data_2;
geometry_msgs::Wrench force_data_3;
geometry_msgs::Wrench force_data_4;
bool Force_connect_flag = false;

// Gripper state storage
std_msgs::Float64MultiArray gripper_state_data;
bool Gripper_connect_flag = true;

// robot control flag
int control_flag = 0;

// 插值相关变量
std::vector<std::vector<double>> q_temp;  // 插值起点
bool start_interp = true;
int interp_step = 0;

bool isSimulation = false;  // 是否为仿真模式

/**********************************************IMU接受数据**************************************************************/
void imuCallback(const sensor_msgs::Imu::ConstPtr &imu)
{
    if (IMU_connect_flag == true)
    {
        imu_data = *imu;  // 保存接收到的数据
    }
}

/**********************************************六维力传感器接受数据*******************************************************/
// Sensor 1
void forceTorqueCallback1(const geometry_msgs::Wrench::ConstPtr &msg)
{
    if (Force_connect_flag = true)
    {
        force_data_1 = *msg;
    }
}

// Sensor 2
void forceTorqueCallback2(const geometry_msgs::Wrench::ConstPtr &msg)
{
    if (Force_connect_flag = true)
    {
        force_data_2 = *msg;
    }
}

// Sensor 3
void forceTorqueCallback3(const geometry_msgs::Wrench::ConstPtr &msg)
{
    if (Force_connect_flag = true)
    {
        force_data_3 = *msg;
    }
}

// Sensor 4
void forceTorqueCallback4(const geometry_msgs::Wrench::ConstPtr &msg)
{
    if (Force_connect_flag = true)
    {
        force_data_4 = *msg;
    }
}

/**********************************************夹爪实际状态订阅者*******************************************************/
void gripperCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if (Gripper_connect_flag == true)
    {
        gripper_state_data = *msg;  // 保存接收到的数据
        for (int branchi = 0; branchi < BRANCHN_N; branchi++)
        {
            q_recv[branchi][MOTOR_BRANCHN_N - 1] = gripper_state_data.data[branchi];  // 更新夹爪状态
        }
    }
}

/******************************************controlFlagCallback*******************************************************/
void controlFlagCallback(const std_msgs::Int32::ConstPtr &msg)
{
    control_flag = msg->data;
    ROS_INFO("Received control_flag: %d", control_flag);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_param("robot_control");

    if (!isSimulation)
    {
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
    }

    // 电机位置发布者
    ros::Publisher motor_state_pub = nh.advertise<std_msgs::Float64MultiArray>("/motor_state", 10);

    // 夹爪控制指令发布者
    ros::Publisher gripper_pub = nh.advertise<std_msgs::Float64MultiArray>("/gripper_command", 10);

    // 夹爪实际状态订阅者
    ros::Subscriber gripper_sub = nh.subscribe<std_msgs::Float64MultiArray>("/gripper_state", 10, gripperCallback);

    // 订阅IMU的消息
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, imuCallback);

    // 订阅4个六维力传感器数据
    ros::Subscriber force_torque_sub_1 = nh.subscribe<geometry_msgs::Wrench>("/force_torque_data_1", 10, forceTorqueCallback1);
    ros::Subscriber force_torque_sub_2 = nh.subscribe<geometry_msgs::Wrench>("/force_torque_data_2", 10, forceTorqueCallback2);
    ros::Subscriber force_torque_sub_3 = nh.subscribe<geometry_msgs::Wrench>("/force_torque_data_3", 10, forceTorqueCallback3);
    ros::Subscriber force_torque_sub_4 = nh.subscribe<geometry_msgs::Wrench>("/force_torque_data_4", 10, forceTorqueCallback4);

    // 订阅控制指令
    ros::Subscriber control_flag_sub = nh.subscribe<std_msgs::Int32>("/control_flag", 10, controlFlagCallback);

    // control_flag = 0 读取上电状态
    // control_flag = 1 运动到初始状态
    // control_flag = 2 抓取演示

    ros::Rate loop_rate(200);  // 200Hz

    while (ros::ok())
    {
        if (control_flag == 0)
        {
            if (!isSimulation)
            {
                // 读取上电状态
                Motor_Rec_Func_ALL();

                // 发布夹爪指令
                std_msgs::Float64MultiArray gripper_command;
                gripper_command.data = {0.0, 0.0, 0.0, 0.0};
                gripper_pub.publish(gripper_command);

                // 保存初始状态
                // q_init = q_recv;
            }
            // 发布电机位置状态
            std_msgs::Float64MultiArray motor_state;
            motor_state.data.resize(BRANCHN_N * MOTOR_BRANCHN_N);

            for (int branchi = 0; branchi < BRANCHN_N; branchi++)
            {
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N; motorj++)
                {
                    motor_state.data[branchi * MOTOR_BRANCHN_N + motorj] = q_recv[branchi][motorj];
                }
            }

            motor_state_pub.publish(motor_state);
        }
        else if (control_flag == 1)
        {
            if (start_interp)
            {
                q_temp.resize(BRANCHN_N, std::vector<double>(MOTOR_BRANCHN_N, 0.0));
                q_temp = q_recv;  // 保存当前位置作为插值起点
                interp_step = 0;
                start_interp = false;
            }

            const int total_steps = 2000;  // 10秒，200Hz
            double ratio = static_cast<double>(interp_step) / total_steps;

            // 插值计算 q_send = q_temp + ratio * (q_init - q_temp)
            for (int i = 0; i < BRANCHN_N; ++i)
            {
                for (int j = 0; j < MOTOR_BRANCHN_N; ++j)
                {
                    double start_val = q_temp[i][j];
                    double target_val = q_init[i][j];
                    q_send[i][j] = start_val + ratio * (target_val - start_val);
                }
            }
            if (!isSimulation)
            {
                Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
            }

            if (++interp_step >= total_steps)
            {
                q_send = q_init;            // 最后一步强制对齐
                interp_step = total_steps;  // 防止溢出
            }
            // 发布电机位置状态
            std_msgs::Float64MultiArray motor_state;
            motor_state.data.resize(BRANCHN_N * MOTOR_BRANCHN_N);

            for (int branchi = 0; branchi < BRANCHN_N; branchi++)
            {
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N; motorj++)
                {
                    motor_state.data[branchi * MOTOR_BRANCHN_N + motorj] = q_send[branchi][motorj];
                }
            }

            motor_state_pub.publish(motor_state);
        }
        else if (control_flag == 2)
        {
            // 抓取演示逻辑可以继续在这里实现
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}