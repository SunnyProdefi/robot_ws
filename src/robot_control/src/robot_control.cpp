#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <robot_planning/PlanPath.h>
#include <robot_planning/RobotPose.h>
#include <robot_planning/CartesianInterpolation.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "motor_driver.h"
#include "ikfast_utils.h"
#include "ikfast_wrapper_single_arm.h"
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include "robot_control/GetBaseLinkPose.h"
#include <Eigen/Dense>
#include <algorithm>

static robots::Kinematics kin_arm3;

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
ros::ServiceClient pose_client;
ros::ServiceClient interp_client;
ros::Publisher motor_state_pub;

// 在外部定义一个静态变量来计数
static int control_flag_3_counter = 0;

// float_base位置
std::vector<double> float_base_position = {0, 0, 0.45, 0, 0.7071, 0, 0.7071};

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

inline void vecToMat4(const std::vector<double> &tf7, Eigen::Matrix4d &M)
{
    Eigen::Vector3d p(tf7[0], tf7[1], tf7[2]);
    Eigen::Quaterniond q(tf7[6], tf7[3], tf7[4], tf7[5]);  // w, x, y, z
    q.normalize();
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = q.toRotationMatrix();
    iso.translation() = p;
    M = iso.matrix();
}

/**
 * @brief  查询分支2、分支3末端在各自 link*_0 坐标系下的 4×4 变换
 * @param[out] T2  link2_0 -> branch2_end
 * @param[out] T3  link3_0 -> branch3_end
 * @return     true 成功, false 失败
 *
 * 依赖全局变量：
 *   - pose_client        (ros::ServiceClient) 调用 /robot_pose
 *   - float_base_position
 *   - q_recv[1], q_recv[2]  当前两分支 6 个关节角
 */
bool getCurrentEEPose(Eigen::Matrix4d &T2, Eigen::Matrix4d &T3)
{
    robot_planning::RobotPose srv;
    srv.request.float_base_pose = float_base_position;
    srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
    srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);

    /* ---------- Branch 2 ---------- */
    srv.request.source_frame = "link2_0";
    srv.request.target_frame = "branch2_end";
    if (!pose_client.call(srv) || !srv.response.success)
    {
        ROS_ERROR("获取 link2_0 → branch2_end 位姿失败: %s", srv.response.message.c_str());
        return false;
    }
    vecToMat4(srv.response.transform, T2);

    /* ---------- Branch 3 ---------- */
    srv.request.source_frame = "link3_0";
    srv.request.target_frame = "branch3_end";
    if (!pose_client.call(srv) || !srv.response.success)
    {
        ROS_ERROR("获取 link3_0 → branch3_end 位姿失败: %s", srv.response.message.c_str());
        return false;
    }
    vecToMat4(srv.response.transform, T3);

    return true;
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
    if (Force_connect_flag == true)
    {
        force_data_1 = *msg;
    }
}

// Sensor 2
void forceTorqueCallback2(const geometry_msgs::Wrench::ConstPtr &msg)
{
    if (Force_connect_flag == true)
    {
        force_data_2 = *msg;
    }
}

// Sensor 3
void forceTorqueCallback3(const geometry_msgs::Wrench::ConstPtr &msg)
{
    if (Force_connect_flag == true)
    {
        force_data_3 = *msg;
    }
}

// Sensor 4
void forceTorqueCallback4(const geometry_msgs::Wrench::ConstPtr &msg)
{
    if (Force_connect_flag == true)
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
    motor_state_pub = nh.advertise<std_msgs::Float64MultiArray>("/motor_state", 10);

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
    pose_client = nh.serviceClient<robot_planning::RobotPose>("/robot_pose");

    // 创建插值服务客户端
    interp_client = nh.serviceClient<robot_planning::CartesianInterpolation>("/cartesian_interpolation");

    // control_flag = 0 读取上电状态

    /* --- 把 4×4 变换矩阵转成 7D pose (x y z qx qy qz qw) --- */
    auto matToPose = [](const Eigen::Matrix4d &M)
    {
        Eigen::Vector3d p = M.block<3, 1>(0, 3);
        Eigen::Quaterniond q(M.block<3, 3>(0, 0));
        q.normalize();
        return std::vector<double>{p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()};
    };

    /* --- 统一调用 cartesian_interpolation --- */
    auto planBranch = [&](int id, const std::vector<double> &q_init, const std::vector<double> &start, const std::vector<double> &goal, double dur, std::vector<double> &out)
    {
        robot_planning::CartesianInterpolation srv;
        srv.request.branch_id = id;
        srv.request.joint_angles = q_init;
        srv.request.start_pose = start;
        srv.request.goal_pose = goal;
        srv.request.duration = dur;
        srv.request.frequency = 200.0;  // 固定 200 Hz
        if (!interp_client.call(srv) || !srv.response.success)
            throw std::runtime_error("Branch " + std::to_string(id) + " 插值失败：" + srv.response.message);
        out = std::move(srv.response.joint_trajectory);
    };

    /* --- 把两分支的关节轨迹合并成 planned_joint_trajectory --- */
    auto mergeTraj = [&](const std::vector<double> &traj2, const std::vector<double> &traj3, std::vector<std::vector<double>> &merged)
    {
        const int n_j = MOTOR_BRANCHN_N - 1;
        size_t min_pts = std::min(traj2.size(), traj3.size()) / n_j;
        merged.clear();
        for (size_t i = 0; i < min_pts; ++i)
        {
            std::vector<double> pt(BRANCHN_N * n_j, 0.0);
            for (int j = 0; j < n_j; ++j)
            {
                pt[1 * n_j + j] = traj2[i * n_j + j];
                pt[2 * n_j + j] = traj3[i * n_j + j];
            }
            for (int b = 0; b < BRANCHN_N; ++b)
                if (b != 1 && b != 2)
                    for (int j = 0; j < n_j; ++j) pt[b * n_j + j] = q_recv[b][j];
            merged.push_back(pt);
        }
    };

    /* --- 单步执行 merged 轨迹（保持原发布 / 发送逻辑） --- */
    auto executeStep = [&](size_t idx)
    {
        const int n_j = MOTOR_BRANCHN_N - 1;
        for (int j = 0; j < n_j; ++j)
        {
            q_send[1][j] = planned_joint_trajectory[idx][1 * n_j + j];
            q_send[2][j] = planned_joint_trajectory[idx][2 * n_j + j];
        }
        if (!isSimulation)
        {
            Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
        }
        else
        {
            for (int j = 0; j < n_j; ++j)
            {
                q_recv[1][j] = q_send[1][j];
                q_recv[2][j] = q_send[2][j];
            }
        }
        std_msgs::Float64MultiArray motor_state;
        motor_state.data.resize(BRANCHN_N * MOTOR_BRANCHN_N);
        for (int b = 0; b < BRANCHN_N; ++b)
            for (int j = 0; j < MOTOR_BRANCHN_N; ++j) motor_state.data[b * MOTOR_BRANCHN_N + j] = q_recv[b][j];
        motor_state_pub.publish(motor_state);
    };

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

            // 发布浮动基座位置
            geometry_msgs::Pose float_base_pose;
            float_base_pose.position.x = float_base_position[0];
            float_base_pose.position.y = float_base_position[1];
            float_base_pose.position.z = float_base_position[2];
            float_base_pose.orientation.x = float_base_position[3];
            float_base_pose.orientation.y = float_base_position[4];
            float_base_pose.orientation.z = float_base_position[5];
            float_base_pose.orientation.w = float_base_position[6];
            float_base_pub.publish(float_base_pose);
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

            const int total_steps = 200;  // 10秒，200Hz
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
                control_flag = 2;

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
                control_flag = 3;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 3)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 3 received" << std::endl;

                // 读取YAML中的世界→物体，base→link2_0 变换
                Eigen::Matrix4d tf_mat_world_obj = loadTransformFromYAML(common_tf_path, "tf_mat_world_obj");
                // std::cout << "tf_mat_world_obj:\n" << tf_mat_world_obj << std::endl;
                Eigen::Matrix4d tf_mat_base_link2_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link2_0");
                // std::cout << "tf_mat_base_link2_0:\n" << tf_mat_base_link2_0 << std::endl;

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

                // std::cout << "tf_mat_world_base:\n" << tf_mat_world_base << std::endl;
                // std::cout << "tf_mat_base_obj:\n" << tf_mat_base_obj << std::endl;
                // std::cout << "tf_mat_link2_0_obj:\n" << tf_mat_link2_0_obj << std::endl;

                // 请求获取分支2末端的位姿（相对于link2_0)）（link2_0 -> flan2）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "link2_0";
                srv.request.target_frame = "branch2_end";

                Eigen::Matrix4d tf_mat_link3_0_flan3;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;
                        // std::cout << "[link2_0 → branch2_end] Transform (xyz + quat):\n";
                        // std::cout << "Position: [" << transform[0] << ", " << transform[1] << ", " << transform[2] << "]\n";
                        // std::cout << "Orientation (quat): [" << transform[3] << ", " << transform[4] << ", " << transform[5] << ", " << transform[6] << "]\n";

                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();                                                                 // 防止精度误差

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        // 转换为 Matrix4d 显示
                        tf_mat_link3_0_flan3 = tf_iso.matrix();
                        // std::cout << "tf_mat_link3_0_flan3:\n" << tf_mat_link3_0_flan3 << std::endl;
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

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position = tf_mat_link2_0_obj.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat(tf_mat_link2_0_obj.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion = quat.coeffs();                     // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> goal_pose = {
                    position.x(),   position.y(),   position.z(),                   // [x, y, z]
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()  // [qx, qy, qz, qw]
                };

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position_flan2 = tf_mat_link3_0_flan3.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link3_0_flan3.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                robot_planning::CartesianInterpolation interp_srv;
                interp_srv.request.branch_id = 2;

                // 设置初始关节角度（float64[]）
                interp_srv.request.joint_angles.assign(q_recv[1].begin(), q_recv[1].begin() + 6);

                interp_srv.request.start_pose = start_pose;
                interp_srv.request.goal_pose = goal_pose;

                // 设置插值参数
                interp_srv.request.duration = 3.0;     // 2 秒
                interp_srv.request.frequency = 200.0;  // 每秒 50 帧

                // 发送请求
                if (interp_client.call(interp_srv))
                {
                    if (interp_srv.response.success)
                    {
                        ROS_INFO_STREAM("Interpolation success: " << interp_srv.response.message);
                        ROS_INFO("Trajectory point count: %lu", interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size());

                        // for (int i = 0; i < interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size(); ++i)
                        // {
                        //     std::cout << "Trajectory point " << i << ": ";
                        //     for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                        //     {
                        //         std::cout << interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j] << " ";
                        //     }
                        //     std::cout << std::endl;
                        // }

                        // 保存轨迹
                        planned_joint_trajectory.clear();
                        int num_points = interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size();
                        for (int i = 0; i < num_points; ++i)
                        {
                            // 创建一个包含所有分支的轨迹点
                            std::vector<double> point(BRANCHN_N * (MOTOR_BRANCHN_N - 1), 0.0);

                            // 复制当前分支的轨迹点
                            for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                            {
                                point[1 * (MOTOR_BRANCHN_N - 1) + j] = interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j];
                            }

                            // 保持其他分支的关节角度不变
                            for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                            {
                                if (branchi != 1)  // 不是当前分支
                                {
                                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                                    {
                                        point[branchi * (MOTOR_BRANCHN_N - 1) + motorj] = q_recv[branchi][motorj];
                                    }
                                }
                            }

                            planned_joint_trajectory.push_back(point);
                        }
                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Interpolation failed: " << interp_srv.response.message);
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service cartesian_interpolation");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                {
                    q_send[1][motorj] = planned_joint_trajectory[trajectory_index][1 * (MOTOR_BRANCHN_N - 1) + motorj];
                }

                // for (int i = 0; i < 6; i++)
                // {
                //     std::cout << q_send[1][i] << " ";
                // }
                // std::cout << std::endl;

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[1][motorj] = q_send[1][motorj];
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
                control_flag = 4;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;

                // 发布夹爪指令
                std_msgs::Float64MultiArray gripper_command;
                gripper_command.data = {0.0, 0.0, 0.5, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.0;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 0.0;
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
        }

        else if (control_flag == 4)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 4 received" << std::endl;

                // 请求获取分支2末端的位姿（相对于link2_0)）（link2_0 -> flan2）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "link2_0";
                srv.request.target_frame = "branch2_end";

                Eigen::Matrix4d tf_mat_link2_0_flan2;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;
                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();                                                                 // 防止精度误差

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        // 转换为 Matrix4d 显示
                        tf_mat_link2_0_flan2 = tf_iso.matrix();
                        // std::cout << "tf_mat_link3_0_flan3:\n" << tf_mat_link2_0_flan2 << std::endl;
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

                Eigen::Matrix4d tf_mat_link2_0_flan2_goal = tf_mat_link2_0_flan2;

                // 构造平移矩阵
                Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
                translation(2, 3) = -0.115;

                // 执行乘法操作，相当于在当前变换的基础上后乘一个纯平移
                tf_mat_link2_0_flan2_goal = tf_mat_link2_0_flan2 * translation;

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position = tf_mat_link2_0_flan2_goal.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat(tf_mat_link2_0_flan2_goal.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion = quat.coeffs();                            // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> goal_pose = {
                    position.x(),   position.y(),   position.z(),                   // [x, y, z]
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()  // [qx, qy, qz, qw]
                };

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position_flan2 = tf_mat_link2_0_flan2.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link2_0_flan2.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                robot_planning::CartesianInterpolation interp_srv;
                interp_srv.request.branch_id = 2;

                // 设置初始关节角度（float64[]）
                interp_srv.request.joint_angles.assign(q_recv[1].begin(), q_recv[1].begin() + 6);

                interp_srv.request.start_pose = start_pose;
                interp_srv.request.goal_pose = goal_pose;

                // 设置插值参数
                interp_srv.request.duration = 3.0;     // 2 秒
                interp_srv.request.frequency = 200.0;  // 每秒 50 帧

                // 发送请求
                if (interp_client.call(interp_srv))
                {
                    if (interp_srv.response.success)
                    {
                        ROS_INFO_STREAM("Interpolation success: " << interp_srv.response.message);
                        ROS_INFO("Trajectory point count: %lu", interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size());

                        // 保存轨迹
                        planned_joint_trajectory.clear();
                        int num_points = interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size();
                        for (int i = 0; i < num_points; ++i)
                        {
                            // 创建一个包含所有分支的轨迹点
                            std::vector<double> point(BRANCHN_N * (MOTOR_BRANCHN_N - 1), 0.0);

                            // 复制当前分支的轨迹点
                            for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                            {
                                point[1 * (MOTOR_BRANCHN_N - 1) + j] = interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j];
                            }

                            // 保持其他分支的关节角度不变
                            for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                            {
                                if (branchi != 1)  // 不是当前分支
                                {
                                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                                    {
                                        point[branchi * (MOTOR_BRANCHN_N - 1) + motorj] = q_recv[branchi][motorj];
                                    }
                                }
                            }

                            planned_joint_trajectory.push_back(point);
                        }
                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Interpolation failed: " << interp_srv.response.message);
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service cartesian_interpolation");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                {
                    q_send[1][motorj] = planned_joint_trajectory[trajectory_index][1 * (MOTOR_BRANCHN_N - 1) + motorj];
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[1][motorj] = q_send[1][motorj];
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
                control_flag = 5;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 5)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 5 received" << std::endl;

                // 读取YAML中的世界→物体，base→link2_0 变换
                Eigen::Matrix4d tf_mat_world_obj_1 = loadTransformFromYAML(common_tf_path, "tf_mat_world_obj_1");
                // std::cout << "tf_mat_world_obj:\n" << tf_mat_world_obj_1 << std::endl;
                Eigen::Matrix4d tf_mat_base_link2_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link2_0");
                // std::cout << "tf_mat_base_link2_0:\n" << tf_mat_base_link2_0 << std::endl;

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

                // base → obj1
                Eigen::Matrix4d tf_mat_base_obj_1 = tf_mat_world_base.inverse() * tf_mat_world_obj_1;

                // link2_0 → obj1
                Eigen::Matrix4d tf_mat_link3_0_flan3_goal = tf_mat_base_link2_0.inverse() * tf_mat_base_obj_1;

                // 请求获取分支2末端的位姿（相对于link2_0)）（link2_0 -> flan2）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "link2_0";
                srv.request.target_frame = "branch2_end";

                Eigen::Matrix4d tf_mat_link3_0_flan3;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;

                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();                                                                 // 防止精度误差

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        // 转换为 Matrix4d 显示
                        tf_mat_link3_0_flan3 = tf_iso.matrix();
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

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position = tf_mat_link3_0_flan3_goal.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat(tf_mat_link3_0_flan3_goal.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion = quat.coeffs();                            // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> goal_pose = {
                    position.x(),   position.y(),   position.z(),                   // [x, y, z]
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()  // [qx, qy, qz, qw]
                };

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position_flan2 = tf_mat_link3_0_flan3.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link3_0_flan3.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                robot_planning::CartesianInterpolation interp_srv;
                interp_srv.request.branch_id = 2;

                // 设置初始关节角度（float64[]）
                interp_srv.request.joint_angles.assign(q_recv[1].begin(), q_recv[1].begin() + 6);

                interp_srv.request.start_pose = start_pose;
                interp_srv.request.goal_pose = goal_pose;

                // 设置插值参数
                interp_srv.request.duration = 3.0;     // 2 秒
                interp_srv.request.frequency = 200.0;  // 每秒 50 帧

                // 发送请求
                if (interp_client.call(interp_srv))
                {
                    if (interp_srv.response.success)
                    {
                        ROS_INFO_STREAM("Interpolation success: " << interp_srv.response.message);
                        ROS_INFO("Trajectory point count: %lu", interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size());

                        // 保存轨迹
                        planned_joint_trajectory.clear();
                        int num_points = interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size();
                        for (int i = 0; i < num_points; ++i)
                        {
                            // 创建一个包含所有分支的轨迹点
                            std::vector<double> point(BRANCHN_N * (MOTOR_BRANCHN_N - 1), 0.0);

                            // 复制当前分支的轨迹点
                            for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                            {
                                point[1 * (MOTOR_BRANCHN_N - 1) + j] = interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j];
                            }

                            // 保持其他分支的关节角度不变
                            for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                            {
                                if (branchi != 1)  // 不是当前分支
                                {
                                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                                    {
                                        point[branchi * (MOTOR_BRANCHN_N - 1) + motorj] = q_recv[branchi][motorj];
                                    }
                                }
                            }

                            planned_joint_trajectory.push_back(point);
                        }
                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Interpolation failed: " << interp_srv.response.message);
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service cartesian_interpolation");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                {
                    q_send[1][motorj] = planned_joint_trajectory[trajectory_index][1 * (MOTOR_BRANCHN_N - 1) + motorj];
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[1][motorj] = q_send[1][motorj];
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
                control_flag = 6;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 6)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 6 received" << std::endl;

                // 请求获取分支3末端的位姿（相对于link3_0)）（link3_0 -> flan3）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "link3_0";
                srv.request.target_frame = "branch3_end";

                Eigen::Matrix4d tf_mat_link3_0_flan3;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;

                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();                                                                 // 防止精度误差

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        // 转换为 Matrix4d 显示
                        tf_mat_link3_0_flan3 = tf_iso.matrix();
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

                Eigen::Matrix4d tf_mat_link3_0_flan3_goal = tf_mat_link3_0_flan3;

                // 构造平移矩阵
                Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
                translation(2, 3) = 0.115;

                // 执行乘法操作，相当于在当前变换的基础上后乘一个纯平移
                tf_mat_link3_0_flan3_goal = tf_mat_link3_0_flan3 * translation;

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position = tf_mat_link3_0_flan3_goal.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat(tf_mat_link3_0_flan3_goal.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion = quat.coeffs();                            // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> goal_pose = {
                    position.x(),   position.y(),   position.z(),                   // [x, y, z]
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()  // [qx, qy, qz, qw]
                };

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position_flan2 = tf_mat_link3_0_flan3.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link3_0_flan3.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                robot_planning::CartesianInterpolation interp_srv;
                interp_srv.request.branch_id = 2;

                // 设置初始关节角度（float64[]）
                interp_srv.request.joint_angles.assign(q_recv[2].begin(), q_recv[2].begin() + 6);

                interp_srv.request.start_pose = start_pose;
                interp_srv.request.goal_pose = goal_pose;

                // 设置插值参数
                interp_srv.request.duration = 3.0;     // 2 秒
                interp_srv.request.frequency = 200.0;  // 每秒 50 帧

                // 发送请求
                if (interp_client.call(interp_srv))
                {
                    if (interp_srv.response.success)
                    {
                        ROS_INFO_STREAM("Interpolation success: " << interp_srv.response.message);
                        ROS_INFO("Trajectory point count: %lu", interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size());

                        // 保存轨迹
                        planned_joint_trajectory.clear();
                        int num_points = interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size();
                        for (int i = 0; i < num_points; ++i)
                        {
                            // 创建一个包含所有分支的轨迹点
                            std::vector<double> point(BRANCHN_N * (MOTOR_BRANCHN_N - 1), 0.0);

                            // 复制当前分支的轨迹点
                            for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                            {
                                point[2 * (MOTOR_BRANCHN_N - 1) + j] = interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j];
                            }

                            // 保持其他分支的关节角度不变
                            for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                            {
                                if (branchi != 2)  // 不是当前分支
                                {
                                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                                    {
                                        point[branchi * (MOTOR_BRANCHN_N - 1) + motorj] = q_recv[branchi][motorj];
                                    }
                                }
                            }

                            planned_joint_trajectory.push_back(point);
                        }
                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Interpolation failed: " << interp_srv.response.message);
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service cartesian_interpolation");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                {
                    q_send[2][motorj] = planned_joint_trajectory[trajectory_index][2 * (MOTOR_BRANCHN_N - 1) + motorj];
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[2][motorj] = q_send[2][motorj];
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
                control_flag = 7;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;

                // 发布夹爪指令
                std_msgs::Float64MultiArray gripper_command;
                gripper_command.data = {0.0, 0.5, 0.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.0;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 0.5;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 0.0;
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
        }

        else if (control_flag == 7)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 7 received" << std::endl;

                // 请求获取分支2末端的位姿（相对于link2_0)）（link2_0 -> flan2）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "link2_0";
                srv.request.target_frame = "branch2_end";

                Eigen::Matrix4d tf_mat_link2_0_flan2;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;

                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();                                                                 // 防止精度误差

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        // 转换为 Matrix4d 显示
                        tf_mat_link2_0_flan2 = tf_iso.matrix();
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

                Eigen::Matrix4d tf_mat_link2_0_flan2_goal = tf_mat_link2_0_flan2;

                // 构造平移矩阵
                Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
                translation(2, 3) = -0.115;

                // 执行乘法操作，相当于在当前变换的基础上后乘一个纯平移
                tf_mat_link2_0_flan2_goal = tf_mat_link2_0_flan2 * translation;

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position = tf_mat_link2_0_flan2_goal.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat(tf_mat_link2_0_flan2_goal.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion = quat.coeffs();                            // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> goal_pose = {
                    position.x(),   position.y(),   position.z(),                   // [x, y, z]
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()  // [qx, qy, qz, qw]
                };

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position_flan2 = tf_mat_link2_0_flan2.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link2_0_flan2.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                robot_planning::CartesianInterpolation interp_srv;
                interp_srv.request.branch_id = 2;

                // 设置初始关节角度（float64[]）
                interp_srv.request.joint_angles.assign(q_recv[1].begin(), q_recv[1].begin() + 6);

                interp_srv.request.start_pose = start_pose;
                interp_srv.request.goal_pose = goal_pose;

                // 设置插值参数
                interp_srv.request.duration = 3.0;     // 2 秒
                interp_srv.request.frequency = 200.0;  // 每秒 50 帧

                // 发送请求
                if (interp_client.call(interp_srv))
                {
                    if (interp_srv.response.success)
                    {
                        ROS_INFO_STREAM("Interpolation success: " << interp_srv.response.message);
                        ROS_INFO("Trajectory point count: %lu", interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size());

                        // 保存轨迹
                        planned_joint_trajectory.clear();
                        int num_points = interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size();
                        for (int i = 0; i < num_points; ++i)
                        {
                            // 创建一个包含所有分支的轨迹点
                            std::vector<double> point(BRANCHN_N * (MOTOR_BRANCHN_N - 1), 0.0);

                            // 复制当前分支的轨迹点
                            for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                            {
                                point[1 * (MOTOR_BRANCHN_N - 1) + j] = interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j];
                            }

                            // 保持其他分支的关节角度不变
                            for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                            {
                                if (branchi != 1)  // 不是当前分支
                                {
                                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                                    {
                                        point[branchi * (MOTOR_BRANCHN_N - 1) + motorj] = q_recv[branchi][motorj];
                                    }
                                }
                            }

                            planned_joint_trajectory.push_back(point);
                        }
                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Interpolation failed: " << interp_srv.response.message);
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service cartesian_interpolation");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                {
                    q_send[1][motorj] = planned_joint_trajectory[trajectory_index][1 * (MOTOR_BRANCHN_N - 1) + motorj];
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[1][motorj] = q_send[1][motorj];
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
                control_flag = 8;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 8)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 8 received" << std::endl;

                // 读取YAML中的世界→物体，base→link3_0 变换
                Eigen::Matrix4d tf_mat_world_obj_2 = loadTransformFromYAML(common_tf_path, "tf_mat_world_obj_2");
                Eigen::Matrix4d tf_mat_base_link3_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link3_0");

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

                // base → obj2
                Eigen::Matrix4d tf_mat_base_obj = tf_mat_world_base.inverse() * tf_mat_world_obj_2;

                // link3_0 → obj2
                Eigen::Matrix4d tf_mat_link3_0_obj = tf_mat_base_link3_0.inverse() * tf_mat_base_obj;

                // 请求获取分支3末端的位姿（相对于link3_0)）（link3_0 -> flan3）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "link3_0";
                srv.request.target_frame = "branch3_end";

                Eigen::Matrix4d tf_mat_link3_0_flan3;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;

                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();                                                                 // 防止精度误差

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        // 转换为 Matrix4d 显示
                        tf_mat_link3_0_flan3 = tf_iso.matrix();
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

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position = tf_mat_link3_0_obj.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat(tf_mat_link3_0_obj.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion = quat.coeffs();                     // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> goal_pose = {
                    position.x(),   position.y(),   position.z(),                   // [x, y, z]
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()  // [qx, qy, qz, qw]
                };

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position_flan2 = tf_mat_link3_0_flan3.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link3_0_flan3.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                robot_planning::CartesianInterpolation interp_srv;
                interp_srv.request.branch_id = 2;

                // 设置初始关节角度（float64[]）
                interp_srv.request.joint_angles.assign(q_recv[2].begin(), q_recv[2].begin() + 6);

                interp_srv.request.start_pose = start_pose;
                interp_srv.request.goal_pose = goal_pose;

                // 设置插值参数
                interp_srv.request.duration = 3.0;     // 2 秒
                interp_srv.request.frequency = 200.0;  // 每秒 50 帧

                // 发送请求
                if (interp_client.call(interp_srv))
                {
                    if (interp_srv.response.success)
                    {
                        ROS_INFO_STREAM("Interpolation success: " << interp_srv.response.message);
                        ROS_INFO("Trajectory point count: %lu", interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size());

                        // 保存轨迹
                        planned_joint_trajectory.clear();
                        int num_points = interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size();
                        for (int i = 0; i < num_points; ++i)
                        {
                            // 创建一个包含所有分支的轨迹点
                            std::vector<double> point(BRANCHN_N * (MOTOR_BRANCHN_N - 1), 0.0);

                            // 复制当前分支的轨迹点
                            for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                            {
                                point[2 * (MOTOR_BRANCHN_N - 1) + j] = interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j];
                            }

                            // 保持其他分支的关节角度不变
                            for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                            {
                                if (branchi != 2)  // 不是当前分支
                                {
                                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                                    {
                                        point[branchi * (MOTOR_BRANCHN_N - 1) + motorj] = q_recv[branchi][motorj];
                                    }
                                }
                            }

                            planned_joint_trajectory.push_back(point);
                        }
                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Interpolation failed: " << interp_srv.response.message);
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service cartesian_interpolation");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                {
                    q_send[2][motorj] = planned_joint_trajectory[trajectory_index][2 * (MOTOR_BRANCHN_N - 1) + motorj];
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[2][motorj] = q_send[2][motorj];
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
                control_flag = 9;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 9)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 9 received" << std::endl;

                // 请求获取分支3末端的位姿（相对于link3_0)）（link3_0 -> flan3）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "link3_0";
                srv.request.target_frame = "branch3_end";

                Eigen::Matrix4d tf_mat_link3_0_flan3;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;

                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();                                                                 // 防止精度误差

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        // 转换为 Matrix4d 显示
                        tf_mat_link3_0_flan3 = tf_iso.matrix();
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

                Eigen::Matrix4d tf_mat_link3_0_flan3_goal = tf_mat_link3_0_flan3;

                // 构造平移矩阵
                Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
                translation(0, 3) = 0.1;

                // 执行乘法操作，相当于在当前变换的基础上后乘一个纯平移
                tf_mat_link3_0_flan3_goal = tf_mat_link3_0_flan3 * translation;

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position = tf_mat_link3_0_flan3_goal.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat(tf_mat_link3_0_flan3_goal.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion = quat.coeffs();                            // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> goal_pose = {
                    position.x(),   position.y(),   position.z(),                   // [x, y, z]
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()  // [qx, qy, qz, qw]
                };

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position_flan2 = tf_mat_link3_0_flan3.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link3_0_flan3.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                robot_planning::CartesianInterpolation interp_srv;
                interp_srv.request.branch_id = 2;

                // 设置初始关节角度（float64[]）
                interp_srv.request.joint_angles.assign(q_recv[2].begin(), q_recv[2].begin() + 6);

                interp_srv.request.start_pose = start_pose;
                interp_srv.request.goal_pose = goal_pose;

                // 设置插值参数
                interp_srv.request.duration = 3.0;     // 2 秒
                interp_srv.request.frequency = 200.0;  // 每秒 50 帧

                // 发送请求
                if (interp_client.call(interp_srv))
                {
                    if (interp_srv.response.success)
                    {
                        ROS_INFO_STREAM("Interpolation success: " << interp_srv.response.message);
                        ROS_INFO("Trajectory point count: %lu", interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size());

                        // 保存轨迹
                        planned_joint_trajectory.clear();
                        int num_points = interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size();
                        for (int i = 0; i < num_points; ++i)
                        {
                            // 创建一个包含所有分支的轨迹点
                            std::vector<double> point(BRANCHN_N * (MOTOR_BRANCHN_N - 1), 0.0);

                            // 复制当前分支的轨迹点
                            for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                            {
                                point[2 * (MOTOR_BRANCHN_N - 1) + j] = interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j];
                            }

                            // 保持其他分支的关节角度不变
                            for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                            {
                                if (branchi != 2)  // 不是当前分支
                                {
                                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                                    {
                                        point[branchi * (MOTOR_BRANCHN_N - 1) + motorj] = q_recv[branchi][motorj];
                                    }
                                }
                            }

                            planned_joint_trajectory.push_back(point);
                        }
                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Interpolation failed: " << interp_srv.response.message);
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service cartesian_interpolation");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                {
                    q_send[2][motorj] = planned_joint_trajectory[trajectory_index][2 * (MOTOR_BRANCHN_N - 1) + motorj];
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[2][motorj] = q_send[2][motorj];
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
                control_flag = 10;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;

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
        }

        else if (control_flag == 21)
        {
            /*----------------------------------------------------
             * 0. 参数
             *--------------------------------------------------*/
            const Eigen::Matrix<double, 6, 1> M = (Eigen::Matrix<double, 6, 1>() << 2.0, 2.0, 2.0, 0.2, 0.2, 0.2).finished();
            const Eigen::Matrix<double, 6, 1> B = (Eigen::Matrix<double, 6, 1>() << 40.0, 40.0, 40.0, 3.0, 3.0, 3.0).finished();
            const Eigen::Matrix<double, 6, 1> K = (Eigen::Matrix<double, 6, 1>() << 100.0, 100.0, 100.0, 5.0, 5.0, 5.0).finished();

            const double MAX_TRANSL = 0.02;                 // m
            const double MAX_ANGLE = 8.0 * M_PI / 180.0;    // rad
            const double MAX_POSVEL = 0.30;                 // m/s
            const double MAX_ANGVEL = 45.0 * M_PI / 180.0;  // rad/s

            /*----------------------------------------------------
             * 1. 锁定 T_ref
             *--------------------------------------------------*/
            static Eigen::Isometry3d T_ref = Eigen::Isometry3d::Identity();
            static bool ref_init = false;

            Eigen::Matrix4d T2_cur, T3_cur;
            if (!getCurrentEEPose(T2_cur, T3_cur))
            {
                control_flag = 0;
                return 0;
            }
            Eigen::Isometry3d T_cur = Eigen::Isometry3d::Identity();
            T_cur.matrix() = T3_cur;

            if (!ref_init)
            {
                T_ref = T_cur;
                ref_init = true;
                ROS_INFO("Admittance: reference pose captured.");
            }

            /*----------------------------------------------------
             * 2. 导纳内部状态
             *--------------------------------------------------*/
            static Eigen::Matrix<double, 6, 1> x_int = Eigen::Matrix<double, 6, 1>::Zero();    // 位移
            static Eigen::Matrix<double, 6, 1> x_int_d = Eigen::Matrix<double, 6, 1>::Zero();  // 速度
            static ros::Time last_t = ros::Time::now();

            double dt = (ros::Time::now() - last_t).toSec();
            last_t = ros::Time::now();
            if (dt <= 0.)
                dt = 1e-3;

            /*----------------------------------------------------
             * 3. 力‑扭矩误差
             *--------------------------------------------------*/
            Eigen::Matrix<double, 6, 1> F_des = Eigen::Matrix<double, 6, 1>::Zero();
            Eigen::Matrix<double, 6, 1> F_meas;
            F_meas << force_data_3.force.x, force_data_3.force.y, force_data_3.force.z, force_data_3.torque.x, force_data_3.torque.y, force_data_3.torque.z;
            Eigen::Matrix<double, 6, 1> F_err = F_des - F_meas;

            /*----------------------------------------------------
             * 4. 导纳微分方程
             *--------------------------------------------------*/
            Eigen::Matrix<double, 6, 1> x_dd = (F_err - B.cwiseProduct(x_int_d) - K.cwiseProduct(x_int)).cwiseQuotient(M);

            x_int_d += x_dd * dt;

            /* 限速 */
            x_int_d.head<3>() = x_int_d.head<3>().cwiseMax(-MAX_POSVEL).cwiseMin(MAX_POSVEL);
            x_int_d.tail<3>() = x_int_d.tail<3>().cwiseMax(-MAX_ANGVEL).cwiseMin(MAX_ANGVEL);

            x_int += x_int_d * dt;

            /* 限幅 */
            x_int.head<3>() = x_int.head<3>().cwiseMax(-MAX_TRANSL).cwiseMin(MAX_TRANSL);
            x_int.tail<3>() = x_int.tail<3>().cwiseMax(-MAX_ANGLE).cwiseMin(MAX_ANGLE);

            /*----------------------------------------------------
             * 5. 生成目标位姿
             *--------------------------------------------------*/
            Eigen::Isometry3d T_tgt = T_ref;
            T_tgt.translation() += x_int.head<3>();

            Eigen::Vector3d rot_vec = x_int.tail<3>();
            double ang = rot_vec.norm();
            if (ang > 1e-6)
            {
                T_tgt.linear() = (Eigen::AngleAxisd(ang, rot_vec / ang)).toRotationMatrix() * T_tgt.linear();
            }

            /* ---------- 调试打印 ---------- */
            ROS_INFO_THROTTLE(0.2,
                              "F  [%.2f %.2f %.2f] N | T  [%.2f %.2f %.2f] Nm | "
                              "dx [%.4f %.4f %.4f] m | dR [%.2f %.2f %.2f] deg",
                              F_meas(0), F_meas(1), F_meas(2),  // Fx Fy Fz
                              F_meas(3), F_meas(4), F_meas(5),  // Tx Ty Tz
                              x_int(0), x_int(1), x_int(2),     // Δx Δy Δz
                              x_int(3) * 180.0 / M_PI, x_int(4) * 180.0 / M_PI,
                              x_int(5) * 180.0 / M_PI);  // ΔRx ΔRy ΔRz

            /*----------------------------------------------------
             * 6. IKFast
             *--------------------------------------------------*/
            std::vector<float> ee_pose12 = {(float)T_tgt(0, 0), (float)T_tgt(0, 1), (float)T_tgt(0, 2), (float)T_tgt(0, 3), (float)T_tgt(1, 0), (float)T_tgt(1, 1), (float)T_tgt(1, 2), (float)T_tgt(1, 3), (float)T_tgt(2, 0), (float)T_tgt(2, 1), (float)T_tgt(2, 2), (float)T_tgt(2, 3)};
            std::vector<float> ik_results = kin_arm3.inverse(ee_pose12);

            std::vector<float> q_cur_vec(6);
            for (int j = 0; j < 6; ++j) q_cur_vec[j] = q_recv[2][j];

            auto [valid_sols, best_sol] = findAllSolutions(ik_results, q_cur_vec, 6);

            /*----------------------------------------------------
             * 7. 写入 q_send
             *--------------------------------------------------*/
            if (best_sol.empty())
            {
                ROS_WARN_THROTTLE(1.0, "IKFast 无有效解，保持当前位置");
                for (int j = 0; j < 6; ++j) q_send[2][j] = q_recv[2][j];
            }
            else
            {
                for (int j = 0; j < 6; ++j) q_send[2][j] = best_sol[j];
            }

            /*----------------------------------------------------
             * 8. 下发 / 仿真
             *--------------------------------------------------*/
            if (!isSimulation)
                Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
            else
                for (int j = 0; j < 6; ++j) q_recv[2][j] = q_send[2][j];

            /*----------------------------------------------------
             * 9. 发布 joint state
             *--------------------------------------------------*/
            std_msgs::Float64MultiArray motor_state;
            motor_state.data.resize(BRANCHN_N * MOTOR_BRANCHN_N);
            for (int b = 0; b < BRANCHN_N; ++b)
                for (int j = 0; j < MOTOR_BRANCHN_N; ++j) motor_state.data[b * MOTOR_BRANCHN_N + j] = q_recv[b][j];
            motor_state_pub.publish(motor_state);
        }

        else if (control_flag == 10)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 10 received" << std::endl;

                // 读取YAML中的world→cube_r，base→link3_0 变换
                Eigen::Matrix4d tf_mat_world_cube_r = loadTransformFromYAML(common_tf_path, "tf_mat_world_cube_r");
                Eigen::Matrix4d tf_mat_base_link3_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link3_0");

                // 构造world→base
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

                // base → cube_r
                Eigen::Matrix4d tf_mat_base_cube_r = tf_mat_world_base.inverse() * tf_mat_world_cube_r;

                // link3_0 → cube_r
                Eigen::Matrix4d tf_mat_link3_0_cube_r = tf_mat_base_link3_0.inverse() * tf_mat_base_cube_r;

                // 请求获取分支3末端的位姿（相对于link3_0)）（link3_0 -> flan3）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "link3_0";
                srv.request.target_frame = "branch3_end";

                Eigen::Matrix4d tf_mat_link3_0_flan3;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;

                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();                                                                 // 防止精度误差

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        // 转换为 Matrix4d 显示
                        tf_mat_link3_0_flan3 = tf_iso.matrix();
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

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position = tf_mat_link3_0_cube_r.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat(tf_mat_link3_0_cube_r.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion = quat.coeffs();                        // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> goal_pose = {
                    position.x(),   position.y(),   position.z(),                   // [x, y, z]
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()  // [qx, qy, qz, qw]
                };

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position_flan2 = tf_mat_link3_0_flan3.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link3_0_flan3.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                robot_planning::CartesianInterpolation interp_srv;
                interp_srv.request.branch_id = 2;

                // 设置初始关节角度（float64[]）
                interp_srv.request.joint_angles.assign(q_recv[2].begin(), q_recv[2].begin() + 6);

                interp_srv.request.start_pose = start_pose;
                interp_srv.request.goal_pose = goal_pose;

                // 设置插值参数
                interp_srv.request.duration = 3.0;     // 2 秒
                interp_srv.request.frequency = 200.0;  // 每秒 50 帧

                // 发送请求
                if (interp_client.call(interp_srv))
                {
                    if (interp_srv.response.success)
                    {
                        ROS_INFO_STREAM("Interpolation success: " << interp_srv.response.message);
                        ROS_INFO("Trajectory point count: %lu", interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size());

                        // 保存轨迹
                        planned_joint_trajectory.clear();
                        int num_points = interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size();
                        for (int i = 0; i < num_points; ++i)
                        {
                            // 创建一个包含所有分支的轨迹点
                            std::vector<double> point(BRANCHN_N * (MOTOR_BRANCHN_N - 1), 0.0);

                            // 复制当前分支的轨迹点
                            for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                            {
                                point[2 * (MOTOR_BRANCHN_N - 1) + j] = interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j];
                            }

                            // 保持其他分支的关节角度不变
                            for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                            {
                                if (branchi != 2)  // 不是当前分支
                                {
                                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                                    {
                                        point[branchi * (MOTOR_BRANCHN_N - 1) + motorj] = q_recv[branchi][motorj];
                                    }
                                }
                            }

                            planned_joint_trajectory.push_back(point);
                        }
                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Interpolation failed: " << interp_srv.response.message);
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service cartesian_interpolation");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                {
                    q_send[2][motorj] = planned_joint_trajectory[trajectory_index][2 * (MOTOR_BRANCHN_N - 1) + motorj];
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[2][motorj] = q_send[2][motorj];
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
                control_flag = 11;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 11)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 11 received" << std::endl;

                // 读取YAML中的world→cube_l，base→link2_0 变换
                Eigen::Matrix4d tf_mat_world_cube_l = loadTransformFromYAML(common_tf_path, "tf_mat_world_cube_l");
                Eigen::Matrix4d tf_mat_base_link2_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link2_0");

                // 构造world→base
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

                // base → cube_l
                Eigen::Matrix4d tf_mat_base_cube_l = tf_mat_world_base.inverse() * tf_mat_world_cube_l;

                // link2_0 → cube_l
                Eigen::Matrix4d tf_mat_link2_0_cube_l = tf_mat_base_link2_0.inverse() * tf_mat_base_cube_l;

                // 请求获取分支3末端的位姿（相对于link3_0)）（link3_0 -> flan3）
                robot_planning::RobotPose srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.source_frame = "link2_0";
                srv.request.target_frame = "branch2_end";

                Eigen::Matrix4d tf_mat_link2_0_flan2;

                if (pose_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        std::vector<double> transform = srv.response.transform;

                        // 用 position 和 quaternion 构造 Eigen::Isometry3d
                        Eigen::Vector3d position(transform[0], transform[1], transform[2]);
                        Eigen::Quaterniond quat(transform[6], transform[3], transform[4], transform[5]);  // w, x, y, z
                        quat.normalize();                                                                 // 防止精度误差

                        Eigen::Isometry3d tf_iso = Eigen::Isometry3d::Identity();
                        tf_iso.linear() = quat.toRotationMatrix();
                        tf_iso.translation() = position;

                        // 转换为 Matrix4d 显示
                        tf_mat_link2_0_flan2 = tf_iso.matrix();
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

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position = tf_mat_link2_0_cube_l.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat(tf_mat_link2_0_cube_l.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion = quat.coeffs();                        // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> goal_pose = {
                    position.x(),   position.y(),   position.z(),                   // [x, y, z]
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()  // [qx, qy, qz, qw]
                };

                // 获取变换矩阵中的位置和四元数
                Eigen::Vector3d position_flan2 = tf_mat_link2_0_flan2.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link2_0_flan2.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                robot_planning::CartesianInterpolation interp_srv;
                interp_srv.request.branch_id = 1;

                // 设置初始关节角度（float64[]）
                interp_srv.request.joint_angles.assign(q_recv[1].begin(), q_recv[1].begin() + 6);

                interp_srv.request.start_pose = start_pose;
                interp_srv.request.goal_pose = goal_pose;

                // 设置插值参数
                interp_srv.request.duration = 3.0;     // 2 秒
                interp_srv.request.frequency = 200.0;  // 每秒 50 帧

                // 发送请求
                if (interp_client.call(interp_srv))
                {
                    if (interp_srv.response.success)
                    {
                        ROS_INFO_STREAM("Interpolation success: " << interp_srv.response.message);
                        ROS_INFO("Trajectory point count: %lu", interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size());

                        // 保存轨迹
                        planned_joint_trajectory.clear();
                        int num_points = interp_srv.response.joint_trajectory.size() / interp_srv.request.joint_angles.size();
                        for (int i = 0; i < num_points; ++i)
                        {
                            // 创建一个包含所有分支的轨迹点
                            std::vector<double> point(BRANCHN_N * (MOTOR_BRANCHN_N - 1), 0.0);

                            // 复制当前分支的轨迹点
                            for (int j = 0; j < interp_srv.request.joint_angles.size(); ++j)
                            {
                                point[1 * (MOTOR_BRANCHN_N - 1) + j] = interp_srv.response.joint_trajectory[i * interp_srv.request.joint_angles.size() + j];
                            }

                            // 保持其他分支的关节角度不变
                            for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                            {
                                if (branchi != 1)  // 不是当前分支
                                {
                                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                                    {
                                        point[branchi * (MOTOR_BRANCHN_N - 1) + motorj] = q_recv[branchi][motorj];
                                    }
                                }
                            }

                            planned_joint_trajectory.push_back(point);
                        }
                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Interpolation failed: " << interp_srv.response.message);
                        control_flag = 0;  // 回到初始状态
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service cartesian_interpolation");
                    control_flag = 0;  // 回到初始状态
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                {
                    q_send[1][motorj] = planned_joint_trajectory[trajectory_index][1 * (MOTOR_BRANCHN_N - 1) + motorj];
                }

                // 执行规划轨迹
                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    // 仿真模式直接使用 q_send
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; motorj++)
                    {
                        q_recv[1][motorj] = q_send[1][motorj];
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
                control_flag = 12;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 12)
        {
            if (!planning_requested)
            {
                ROS_INFO("=== flag 12 : Y +0.03 ===");

                /* ---- 当前位姿 ---- */
                Eigen::Matrix4d T2_cur, T3_cur;
                if (!getCurrentEEPose(T2_cur, T3_cur))
                {
                    control_flag = 0;
                    return 0;
                }

                /* ---- 目标位姿 (+Y) ---- */
                Eigen::Matrix4d TYp = Eigen::Matrix4d::Identity();
                TYp(1, 3) = 0.03;
                Eigen::Matrix4d T2_goal = T2_cur * TYp;
                Eigen::Matrix4d T3_goal = T3_cur * TYp;

                /* ---- 规划两分支 ---- */
                std::vector<double> traj2, traj3;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 5}, matToPose(T2_cur), matToPose(T2_goal), 4.0, traj2);
                    planBranch(2, {q_recv[2].begin(), q_recv[2].begin() + 5}, matToPose(T3_cur), matToPose(T3_goal), 4.0, traj3);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeTraj(traj2, traj3, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
            }
            else
            {                       // 完成
                control_flag = 13;  // → 13
                planning_requested = planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 13)
        {
            if (!planning_requested)
            {
                ROS_INFO("=== flag 13 : X ±0.02 ===");

                Eigen::Matrix4d T2_cur, T3_cur;
                if (!getCurrentEEPose(T2_cur, T3_cur))
                {
                    control_flag = 0;
                    return 0;
                }

                Eigen::Matrix4d TZm = Eigen::Matrix4d::Identity();
                TZm(0, 3) = -0.02;  // branch2
                Eigen::Matrix4d TZp = Eigen::Matrix4d::Identity();
                TZp(0, 3) = 0.02;  // branch3
                Eigen::Matrix4d T2_goal = T2_cur * TZm;
                Eigen::Matrix4d T3_goal = T3_cur * TZp;

                std::vector<double> traj2, traj3;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 5}, matToPose(T2_cur), matToPose(T2_goal), 4.0, traj2);
                    planBranch(2, {q_recv[2].begin(), q_recv[2].begin() + 5}, matToPose(T3_cur), matToPose(T3_goal), 4.0, traj3);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeTraj(traj2, traj3, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
            }
            else
            {
                control_flag = 14;  // → 14
                planning_requested = planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 14)
        {
            if (!planning_requested)
            {
                ROS_INFO("=== flag 14 : Z ±0.125 ===");

                Eigen::Matrix4d T2_cur, T3_cur;
                if (!getCurrentEEPose(T2_cur, T3_cur))
                {
                    control_flag = 0;
                    return 0;
                }

                Eigen::Matrix4d TZm = Eigen::Matrix4d::Identity();
                TZm(2, 3) = 0.125;  // branch2
                Eigen::Matrix4d TZp = Eigen::Matrix4d::Identity();
                TZp(2, 3) = -0.125;  // branch3
                Eigen::Matrix4d T2_goal = T2_cur * TZm;
                Eigen::Matrix4d T3_goal = T3_cur * TZp;

                std::vector<double> traj2, traj3;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 5}, matToPose(T2_cur), matToPose(T2_goal), 4.0, traj2);
                    planBranch(2, {q_recv[2].begin(), q_recv[2].begin() + 5}, matToPose(T3_cur), matToPose(T3_goal), 4.0, traj3);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeTraj(traj2, traj3, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
            }
            else
            {
                control_flag = 15;  // → 14
                planning_requested = planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 15)
        {
            if (!planning_requested)
            {
                ROS_INFO("=== flag 15 : Y -0.03 (return) ===");

                Eigen::Matrix4d T2_cur, T3_cur;
                if (!getCurrentEEPose(T2_cur, T3_cur))
                {
                    control_flag = 0;
                    return 0;
                }

                Eigen::Matrix4d TYn = Eigen::Matrix4d::Identity();
                TYn(1, 3) = -0.03;
                Eigen::Matrix4d T2_goal = T2_cur * TYn;
                Eigen::Matrix4d T3_goal = T3_cur * TYn;

                std::vector<double> traj2, traj3;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 5}, matToPose(T2_cur), matToPose(T2_goal), 4.0, traj2);
                    planBranch(2, {q_recv[2].begin(), q_recv[2].begin() + 5}, matToPose(T3_cur), matToPose(T3_goal), 4.0, traj3);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeTraj(traj2, traj3, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
            }
            else
            {
                control_flag = 0;  // 回到空闲
                planning_requested = planning_completed = false;
                trajectory_index = 0;
                ROS_INFO("All 3 segments completed.");
            }
        }

        else if (control_flag == 20)
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

        else if (control_flag == 103)
        {
            // 发布夹爪指令
            std_msgs::Float64MultiArray gripper_command;
            gripper_command.data = {0.0, 0.0, 0.5, 0.0};
            gripper_pub.publish(gripper_command);

            q_recv[0][MOTOR_BRANCHN_N - 1] = 0.0;  // 更新夹爪状态
            q_recv[1][MOTOR_BRANCHN_N - 1] = 0.0;
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

        else if (control_flag == 104)
        {
            // 发布夹爪指令
            std_msgs::Float64MultiArray gripper_command;
            gripper_command.data = {0.0, 0.0, 0.0, 0.0};
            gripper_pub.publish(gripper_command);

            q_recv[0][MOTOR_BRANCHN_N - 1] = 0.0;  // 更新夹爪状态
            q_recv[1][MOTOR_BRANCHN_N - 1] = 0.0;
            q_recv[2][MOTOR_BRANCHN_N - 1] = 0.0;
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