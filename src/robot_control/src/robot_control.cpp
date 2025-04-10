#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <robot_planning/PlanPath.h>
#include <robot_planning/RobotPose.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "motor_driver.h"
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include "robot_control/GetBaseLinkPose.h"
#include <Eigen/Dense>

// IMU data storage
sensor_msgs::Imu imu_data;
bool IMU_connect_flag;

// Force-torque data storage for 4 sensors
geometry_msgs::Wrench force_data_1;
geometry_msgs::Wrench force_data_2;
geometry_msgs::Wrench force_data_3;
geometry_msgs::Wrench force_data_4;
bool Force_connect_flag;

// Gripper state storage
std_msgs::Float64MultiArray gripper_state_data;
bool Gripper_connect_flag;

// robot control flag
int control_flag = 0;

// 插值相关变量
std::vector<std::vector<double>> q_temp;  // 插值起点
bool start_interp = true;
int interp_step = 0;

bool start_interp_end = true;
int interp_step_end = 0;

bool isSimulation;  // 是否为仿真模式

// 运动规划相关变量
bool planning_requested = false;
bool planning_completed = false;
std::vector<std::vector<double>> planned_joint_trajectory;
std::vector<std::vector<double>> floating_base_sequence;
int trajectory_index = 0;
ros::ServiceClient planning_client;

// 在外部定义一个静态变量来计数
static int control_flag_3_counter = 0;

// float_base位置
std::vector<double> float_base_position(7, 0.0);

// yaml路径
std::string common_tf_path = ros::package::getPath("robot_control") + "/config/common_tf.yaml";

// 从YAML文件加载变换矩阵
Eigen::Matrix4d loadTransformFromYAML(const std::string &file_path, const std::string &transform_name)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    try
    {
        YAML::Node config = YAML::LoadFile(file_path);
        if (config[transform_name])
        {
            YAML::Node target_pose = config[transform_name]["target_pose"];
            if (target_pose)
            {
                // 读取位置
                YAML::Node position = target_pose["position"];
                if (position && position.size() == 3)
                {
                    transform(0, 3) = position[0].as<double>();
                    transform(1, 3) = position[1].as<double>();
                    transform(2, 3) = position[2].as<double>();
                }
                else
                {
                    ROS_WARN("Position data is missing or not of size 3 in %s", transform_name.c_str());
                }

                // 读取方向（3x3 旋转矩阵）
                YAML::Node orientation = target_pose["orientation"];
                if (orientation && orientation.size() == 3)
                {
                    for (int i = 0; i < 3; ++i)
                    {
                        YAML::Node row = orientation[i];
                        if (row && row.size() == 3)
                        {
                            for (int j = 0; j < 3; ++j)
                            {
                                transform(i, j) = row[j].as<double>();
                            }
                        }
                        else
                        {
                            ROS_WARN("Orientation row %d in %s is not size 3", i, transform_name.c_str());
                        }
                    }
                }
                else
                {
                    ROS_WARN("Orientation data is missing or not size 3 in %s", transform_name.c_str());
                }
            }
            else
            {
                ROS_WARN("No 'target_pose' field found under %s", transform_name.c_str());
            }
        }
        else
        {
            ROS_WARN("Transform name '%s' not found in YAML file", transform_name.c_str());
        }
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("Failed to load transform from YAML: %s", e.what());
    }

    return transform;
}

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

    // 从参数服务器读取配置参数
    nh_param.param<bool>("imu_connect", IMU_connect_flag, false);
    nh_param.param<bool>("gripper_connect", Gripper_connect_flag, true);
    nh_param.param<bool>("force_connect", Force_connect_flag, false);
    nh_param.param<bool>("simulation_mode", isSimulation, false);

    ROS_INFO("Configuration loaded - IMU: %s, Gripper: %s, Force: %s, Simulation: %s", IMU_connect_flag ? "true" : "false", Gripper_connect_flag ? "true" : "false", Force_connect_flag ? "true" : "false", isSimulation ? "true" : "false");

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

    // floatb_base位置发布者
    ros::Publisher float_base_pub = nh.advertise<geometry_msgs::Pose>("/floating_base_state", 10);

    // 创建服务客户端
    ros::ServiceClient client = nh.serviceClient<robot_control::GetBaseLinkPose>("get_base_link_pose");

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

    // 创建运动规划服务客户端
    planning_client = nh.serviceClient<robot_planning::PlanPath>("/plan_path");

    // 创建 service client
    ros::ServiceClient pose_client = nh.serviceClient<robot_planning::RobotPose>("/robot_pose");

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
            else
            {
                // 在仿真模式下，直接使用 q_send,不管夹爪
                for (int branchi = 0; branchi < BRANCHN_N; branchi++)
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[branchi][motorj] = q_send[branchi][motorj];
                    }
                }
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
                    motor_state.data[branchi * MOTOR_BRANCHN_N + motorj] = q_recv[branchi][motorj];
                }
            }

            motor_state_pub.publish(motor_state);
        }
        else if (control_flag == 2)
        {
            if (!planning_requested)
            {
                // 发送运动规划请求
                robot_planning::PlanPath srv;
                if (planning_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        ROS_INFO("Planning request sent successfully");
                        planning_requested = true;
                    }
                    else
                    {
                        ROS_ERROR("Planning request failed: %s", srv.response.message.c_str());
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call planning service");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (!planning_completed)
            {
                // 使用 ros::package::getPath() 获取路径
                std::string robot_planning_path = ros::package::getPath("robot_planning");
                std::string planning_result_path = robot_planning_path + "/config/planning_result.yaml";

                // 检查规划结果文件是否存在
                std::ifstream file(planning_result_path);
                if (file.good())
                {
                    YAML::Node config = YAML::LoadFile(planning_result_path);

                    // 读取 joint_angle_sequence
                    if (config["joint_angle_sequence"])
                    {
                        planned_joint_trajectory.clear();
                        for (const auto &point : config["joint_angle_sequence"])
                        {
                            std::vector<double> angles;
                            for (const auto &angle : point)
                            {
                                angles.push_back(angle.as<double>());
                            }
                            planned_joint_trajectory.push_back(angles);
                        }
                        ROS_INFO("Loaded %zu joint angle points", planned_joint_trajectory.size());
                    }

                    // 读取 floating_base_sequence
                    if (config["floating_base_sequence"])
                    {
                        floating_base_sequence.clear();
                        for (const auto &pose : config["floating_base_sequence"])
                        {
                            std::vector<double> base_state;
                            for (const auto &val : pose)
                            {
                                base_state.push_back(val.as<double>());
                            }
                            floating_base_sequence.push_back(base_state);
                        }
                        ROS_INFO("Loaded %zu floating base poses", floating_base_sequence.size());
                    }

                    // 成功标志
                    planning_completed = true;
                    trajectory_index = 0;
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int branchi = 0; branchi < BRANCHN_N; branchi++)
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_send[branchi][motorj] = planned_joint_trajectory[trajectory_index][branchi * (MOTOR_BRANCHN_N - 1) + motorj];
                    }
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);

                    // ==== 请求 base_link 位姿 ====
                    robot_control::GetBaseLinkPose srv;

                    // 提取 JOINT1_1 ~ JOINT1_6 和 JOINT4_1 ~ JOINT4_6
                    for (int j = 0; j < 6; ++j)
                    {
                        // 分支1：JOINT1_1 ~ JOINT1_6 在 planned_joint_trajectory 中的索引是 0~5
                        srv.request.joint_angles_branch1.push_back(planned_joint_trajectory[trajectory_index][j]);

                        // 分支4：JOINT4_1 ~ JOINT4_6 在 planned_joint_trajectory 中的索引是 18~23（共24个角）
                        srv.request.joint_angles_branch4.push_back(planned_joint_trajectory[trajectory_index][18 + j]);
                    }

                    if (client.call(srv))
                    {
                        ROS_INFO("Base link pose calculated: Position (x: %f, y: %f, z: %f), Orientation (x: %f, y: %f, z: %f, w: %f)", srv.response.base_link_pose.position.x, srv.response.base_link_pose.position.y, srv.response.base_link_pose.position.z, srv.response.base_link_pose.orientation.x,
                                 srv.response.base_link_pose.orientation.y, srv.response.base_link_pose.orientation.z, srv.response.base_link_pose.orientation.w);

                        // 可选：你可以将此 Pose 发布给 RViz 或浮动基座控制模块
                        float_base_pub.publish(srv.response.base_link_pose);
                        // 更新浮动基座位置
                        float_base_position[0] = srv.response.base_link_pose.position.x;
                        float_base_position[1] = srv.response.base_link_pose.position.y;
                        float_base_position[2] = srv.response.base_link_pose.position.z;
                        float_base_position[3] = srv.response.base_link_pose.orientation.x;
                        float_base_position[4] = srv.response.base_link_pose.orientation.y;
                        float_base_position[5] = srv.response.base_link_pose.orientation.z;
                        float_base_position[6] = srv.response.base_link_pose.orientation.w;
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service get_base_link_pose");
                    }
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int branchi = 0; branchi < BRANCHN_N; branchi++)
                    {
                        for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                        {
                            q_recv[branchi][motorj] = q_send[branchi][motorj];
                        }
                    }

                    // 发布浮动基座位置
                    geometry_msgs::Pose float_base_pose;
                    float_base_pose.position.x = floating_base_sequence[trajectory_index][0];
                    float_base_pose.position.y = floating_base_sequence[trajectory_index][1];
                    float_base_pose.position.z = floating_base_sequence[trajectory_index][2];
                    float_base_pose.orientation.x = floating_base_sequence[trajectory_index][3];
                    float_base_pose.orientation.y = floating_base_sequence[trajectory_index][4];
                    float_base_pose.orientation.z = floating_base_sequence[trajectory_index][5];
                    float_base_pose.orientation.w = floating_base_sequence[trajectory_index][6];
                    float_base_pub.publish(float_base_pose);
                    // // ==== 请求 base_link 位姿 ====
                    // robot_control::GetBaseLinkPose srv;

                    // // 提取 JOINT1_1 ~ JOINT1_6 和 JOINT4_1 ~ JOINT4_6
                    // for (int j = 0; j < 6; ++j)
                    // {
                    //     // 分支1：JOINT1_1 ~ JOINT1_6 在 planned_joint_trajectory 中的索引是 0~5
                    //     srv.request.joint_angles_branch1.push_back(planned_joint_trajectory[trajectory_index][j]);

                    //     // 分支4：JOINT4_1 ~ JOINT4_6 在 planned_joint_trajectory 中的索引是 18~23（共24个角）
                    //     srv.request.joint_angles_branch4.push_back(planned_joint_trajectory[trajectory_index][18 + j]);
                    // }

                    // if (client.call(srv))
                    // {
                    //     ROS_INFO("Base link pose calculated: Position (x: %f, y: %f, z: %f), Orientation (x: %f, y: %f, z: %f, w: %f)", srv.response.base_link_pose.position.x, srv.response.base_link_pose.position.y, srv.response.base_link_pose.position.z,
                    //     srv.response.base_link_pose.orientation.x,
                    //              srv.response.base_link_pose.orientation.y, srv.response.base_link_pose.orientation.z, srv.response.base_link_pose.orientation.w);

                    //     // 可选：你可以将此 Pose 发布给 RViz 或浮动基座控制模块
                    //     float_base_pub.publish(srv.response.base_link_pose);
                    // }
                    // else
                    // {
                    //     ROS_ERROR("Failed to call service get_base_link_pose");
                    // }

                    // 更新浮动基座位置
                    float_base_position[0] = floating_base_sequence[trajectory_index][0];
                    float_base_position[1] = floating_base_sequence[trajectory_index][1];
                    float_base_position[2] = floating_base_sequence[trajectory_index][2];
                    float_base_position[3] = floating_base_sequence[trajectory_index][3];
                    float_base_position[4] = floating_base_sequence[trajectory_index][4];
                    float_base_position[5] = floating_base_sequence[trajectory_index][5];
                    float_base_position[6] = floating_base_sequence[trajectory_index][6];
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

                trajectory_index++;
            }
            else
            {
                ROS_INFO("Trajectory execution completed");
                control_flag = 0;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }
        else if (control_flag == 3)
        {
            if (control_flag_3_counter >= 10)
            {
                control_flag = 0;
                std::cout << "control_flag == 3 已执行10次，已重置 control_flag = 0" << std::endl;
            }
            else
            {
                std::cout << "Control flag 3 received" << std::endl;
                std::cout << "完成抓取" << std::endl;

                // 读取YAML中的世界→物体，base→link2_0 变换
                Eigen::Matrix4d tf_mat_world_obj = loadTransformFromYAML(common_tf_path, "tf_mat_world_obj");
                std::cout << "tf_mat_world_obj:\n" << tf_mat_world_obj << std::endl;
                Eigen::Matrix4d tf_mat_base_link2_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link2_0");
                std::cout << "tf_mat_base_link2_0:\n" << tf_mat_base_link2_0 << std::endl;

                // 构造世界→base
                Eigen::Vector3d trans_base(float_base_position[0], float_base_position[1], float_base_position[2]);
                Eigen::Quaterniond quat_base(float_base_position[6],  // qw
                                             float_base_position[3],  // qx
                                             float_base_position[4],  // qy
                                             float_base_position[5]   // qz
                );
                Eigen::Matrix3d rot_base = quat_base.normalized().toRotationMatrix();
                Eigen::Matrix4d tf_mat_world_base = Eigen::Matrix4d::Identity();
                tf_mat_world_base.block<3, 3>(0, 0) = rot_base;
                tf_mat_world_base.block<3, 1>(0, 3) = trans_base;

                // base → obj
                Eigen::Matrix4d tf_mat_base_obj = tf_mat_world_base.inverse() * tf_mat_world_obj;

                // link2_0 → obj
                Eigen::Matrix4d tf_mat_link2_0_obj = tf_mat_base_link2_0.inverse() * tf_mat_base_obj;

                std::cout << "tf_mat_world_base:\n" << tf_mat_world_base << std::endl;
                std::cout << "tf_mat_base_obj:\n" << tf_mat_base_obj << std::endl;
                std::cout << "tf_mat_link2_0_obj:\n" << tf_mat_link2_0_obj << std::endl;

                // // 请求获取分支2末端的位姿（相对于link2_0)）
                // robot_planning::RobotPose srv;
                // srv.request.float_base_pose = float_base_position;
                // srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                // srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                // srv.request.source_frame = "link2_0";
                // srv.request.target_frame = "branch2_end";

                // Eigen::Matrix4d tf_mat_link2_0_flan2;

                // if (pose_client.call(srv))
                // {
                //     if (srv.response.success)
                //     {
                //         std::vector<double> transform = srv.response.transform;
                //         std::cout << "[link2_0 → branch2_end] Transform (xyz + quat):\n";
                //         std::cout << "Position: [" << transform[0] << ", " << transform[1] << ", " << transform[2] << "]\n";
                //         std::cout << "Orientation (quat): [" << transform[3] << ", " << transform[4] << ", " << transform[5] << ", " << transform[6] << "]\n";

                //         // 用 position 和 quaternion 构造 Eigen::Isometry3d
                //         Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                //         Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                //         quat.normalize();                                                                 // 防止精度误差

                //         Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                //         tf_iso.linear() = quat.toRotationMatrix();
                //         tf_iso.translation() = position;

                //         // 转换为 Matrix4d 显示
                //         Eigen::Matrix4d tf_mat_link2_0_flan2 = tf_iso.matrix();
                //         std::cout << "tf_mat_link2_0_flan2:\n" << tf_mat_link2_0_flan2 << std::endl;
                //     }
                //     else
                //     {
                //         std::cerr << "Service call failed: " << srv.response.message << std::endl;
                //     }
                // }
                // else
                // {
                //     std::cerr << "Failed to call service /robot_pose" << std::endl;
                // }
                // 请求获取分支2末端的位姿（相对于 world）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "world";  // ← 修改这里
                srv.request.target_frame = "branch2_end";

                Eigen::Matrix4d tf_mat_world_branch2;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;
                        std::cout << "[world → branch2_end] Transform (xyz + quat):\n";
                        std::cout << "Position: [" << transform[0] << ", " << transform[1] << ", " << transform[2] << "]\n";
                        std::cout << "Orientation (quat): [" << transform[3] << ", " << transform[4] << ", " << transform[5] << ", " << transform[6] << "]\n";

                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        tf_mat_world_branch2 = tf_iso.matrix();  // 保存最终矩阵
                        std::cout << "tf_mat_world_branch2:\n" << tf_mat_world_branch2 << std::endl;
                    }
                    else
                    {
                        std::cerr << "Service call failed: " << srv.response.message << std::endl;
                    }
                }
                else
                {
                    std::cerr << "Failed to call service /robot_pose" << std::endl;
                }

                // 执行次数加1
                control_flag_3_counter++;
            }
        }
        else if (control_flag == 4)
        {
            cout << "Control flag 4 received" << endl;
            cout << "完成取出" << endl;
        }
        else if (control_flag == 5)
        {
            cout << "Control flag 5 received" << endl;
            cout << "完成交接" << endl;
        }
        else if (control_flag == 6)
        {
            cout << "Control flag 6 received" << endl;
            cout << "完成放置" << endl;
        }
        else if (control_flag == 9)
        {
            if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int branchi = 0; branchi < BRANCHN_N; branchi++)
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_send[branchi][motorj] = planned_joint_trajectory[planned_joint_trajectory.size() - trajectory_index - 1][branchi * (MOTOR_BRANCHN_N - 1) + motorj];
                    }
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int branchi = 0; branchi < BRANCHN_N; branchi++)
                    {
                        for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                        {
                            q_recv[branchi][motorj] = q_send[branchi][motorj];
                        }
                    }
                    // ==== 请求 base_link 位姿 ====
                    robot_control::GetBaseLinkPose srv;

                    // 提取 JOINT1_1 ~ JOINT1_6 和 JOINT4_1 ~ JOINT4_6
                    for (int j = 0; j < 6; ++j)
                    {
                        // 分支1：JOINT1_1 ~ JOINT1_6 在 planned_joint_trajectory 中的索引是 0~5
                        srv.request.joint_angles_branch1.push_back(planned_joint_trajectory[planned_joint_trajectory.size() - trajectory_index - 1][j]);

                        // 分支4：JOINT4_1 ~ JOINT4_6 在 planned_joint_trajectory 中的索引是 18~23（共24个角）
                        srv.request.joint_angles_branch4.push_back(planned_joint_trajectory[planned_joint_trajectory.size() - trajectory_index - 1][18 + j]);
                    }

                    if (client.call(srv))
                    {
                        ROS_INFO("Base link pose calculated: Position (x: %f, y: %f, z: %f), Orientation (x: %f, y: %f, z: %f, w: %f)", srv.response.base_link_pose.position.x, srv.response.base_link_pose.position.y, srv.response.base_link_pose.position.z, srv.response.base_link_pose.orientation.x,
                                 srv.response.base_link_pose.orientation.y, srv.response.base_link_pose.orientation.z, srv.response.base_link_pose.orientation.w);

                        // 可选：你可以将此 Pose 发布给 RViz 或浮动基座控制模块
                        float_base_pub.publish(srv.response.base_link_pose);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service get_base_link_pose");
                    }
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

                trajectory_index++;
            }
            else
            {
                ROS_INFO("Trajectory execution completed");
                control_flag = 0;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }
        else if (control_flag == 101)
        {
            // 发布夹爪指令
            std_msgs::Float64MultiArray gripper_command;
            gripper_command.data = {1.0, 1.0, 1.0, 1.0};
            gripper_pub.publish(gripper_command);

            q_recv[0][MOTOR_BRANCHN_N - 1] = 1.0;  // 更新夹爪状态
            q_recv[1][MOTOR_BRANCHN_N - 1] = 1.0;
            q_recv[2][MOTOR_BRANCHN_N - 1] = 1.0;
            q_recv[3][MOTOR_BRANCHN_N - 1] = 1.0;
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
        else if (control_flag == 102)
        {
            // 发布夹爪指令
            std_msgs::Float64MultiArray gripper_command;
            gripper_command.data = {0.0, 0.5, 0.5, 0.0};
            gripper_pub.publish(gripper_command);

            q_recv[0][MOTOR_BRANCHN_N - 1] = 0.0;  // 更新夹爪状态
            q_recv[1][MOTOR_BRANCHN_N - 1] = 0.5;
            q_recv[2][MOTOR_BRANCHN_N - 1] = 0.5;
            q_recv[3][MOTOR_BRANCHN_N - 1] = 0.0;
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
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}