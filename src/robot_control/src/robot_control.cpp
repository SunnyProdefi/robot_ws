#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <robot_planning/PlanPath.h>
#include <robot_planning/PlanPathHome.h>
#include <robot_planning/PlanDualArmPath.h>
#include <robot_planning/RobotPose.h>
#include <robot_rrt/RRTPlanPath.h>
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
#include <geometry_msgs/Transform.h>

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

double plantime = 2.0;

// 导纳相关
static std::ofstream admittance_log;
static bool log_initialized = false;

// 运动规划相关变量
bool planning_requested = false;
bool planning_completed = false;
std::vector<std::vector<double>> planned_joint_trajectory;
std::vector<std::vector<double>> floating_base_sequence;
int trajectory_index = 0;
ros::ServiceClient planning_client;
ros::ServiceClient pose_client;
ros::ServiceClient interp_client;
ros::ServiceClient planning_client_home;
ros::ServiceClient plan_dual_arm_path_client;
ros::ServiceClient rrt_plan_client;
ros::Publisher motor_state_pub;

std::vector<double> q_init_rrt, q_goal_rrt;

// 在外部定义一个静态变量来计数
static int control_flag_3_counter = 0;

// float_base位置
std::vector<double> float_base_position = {0, 0, 0.45, 0, 0.7071, 0, 0.7071};

// grasp_object位置
std::vector<double> grasp_object_position = {0.5825, 0.36, -0.07, 0, 0, -0.258734, 0.965946};

// yaml路径
std::string common_tf_path = ros::package::getPath("robot_control") + "/config/common_tf.yaml";

std::string csv_file_path = ros::package::getPath("robot_control") + "/data/admittance_data.csv";

Eigen::Matrix4d goal_tf_mat_base_link2_0, goal_tf_mat_base_link3_0;

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

geometry_msgs::Transform eigenToTransformMsg(const Eigen::Matrix4d &mat)
{
    geometry_msgs::Transform tf;
    tf.translation.x = mat(0, 3);
    tf.translation.y = mat(1, 3);
    tf.translation.z = mat(2, 3);

    Eigen::Matrix3d rot = mat.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rot);
    tf.rotation.x = q.x();
    tf.rotation.y = q.y();
    tf.rotation.z = q.z();
    tf.rotation.w = q.w();

    return tf;
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

    // grasp_object位置发布者
    ros::Publisher grasp_object_pub = nh.advertise<geometry_msgs::Pose>("/grasp_object_state", 10);

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

    planning_client_home = nh.serviceClient<robot_planning::PlanPathHome>("/plan_path_home");

    plan_dual_arm_path_client = nh.serviceClient<robot_planning::PlanDualArmPath>("/plan_dual_arm_path");

    rrt_plan_client = nh.serviceClient<robot_rrt::RRTPlanPath>("/rrt_plan_path");

    // 创建 service client
    pose_client = nh.serviceClient<robot_planning::RobotPose>("/robot_pose");

    // 创建插值服务客户端
    interp_client = nh.serviceClient<robot_planning::CartesianInterpolation>("/cartesian_interpolation");

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

    auto mergeBranch2Only = [&](const std::vector<double> &traj2, std::vector<std::vector<double>> &merged)
    {
        const int n_j = MOTOR_BRANCHN_N - 1;  // 每个分支的关节数
        size_t num_pts = traj2.size() / n_j;

        merged.clear();
        for (size_t i = 0; i < num_pts; ++i)
        {
            std::vector<double> pt(BRANCHN_N * n_j, 0.0);

            // 分支2填入轨迹
            for (int j = 0; j < n_j; ++j)
            {
                pt[1 * n_j + j] = traj2[i * n_j + j];  // 注意：分支2的索引是 1
            }

            // 其他分支保持原样
            for (int b = 0; b < BRANCHN_N; ++b)
            {
                if (b != 1)
                {
                    for (int j = 0; j < n_j; ++j)
                    {
                        pt[b * n_j + j] = q_recv[b][j];
                    }
                }
            }
            merged.push_back(pt);
        }
    };

    auto mergeBranch3Only = [&](const std::vector<double> &traj3, std::vector<std::vector<double>> &merged)
    {
        const int n_j = MOTOR_BRANCHN_N - 1;  // 每个分支的关节数
        size_t num_pts = traj3.size() / n_j;

        merged.clear();
        for (size_t i = 0; i < num_pts; ++i)
        {
            std::vector<double> pt(BRANCHN_N * n_j, 0.0);

            // 分支3填入轨迹
            for (int j = 0; j < n_j; ++j)
            {
                pt[2 * n_j + j] = traj3[i * n_j + j];  // 注意：分支3的索引是 2
            }

            // 其他分支保持原样
            for (int b = 0; b < BRANCHN_N; ++b)
            {
                if (b != 2)
                {
                    for (int j = 0; j < n_j; ++j)
                    {
                        pt[b * n_j + j] = q_recv[b][j];
                    }
                }
            }

            merged.push_back(pt);
        }
    };

    auto mergeTrajFromFlat = [&](const std::vector<double> &flat_joint_trajectory, std::vector<std::vector<double>> &merged)
    {
        const int n_j = MOTOR_BRANCHN_N - 1;                                  // 每个分支的关节数
        const size_t total_steps = flat_joint_trajectory.size() / (2 * n_j);  // 每步12维（分支2+3）

        merged.clear();
        for (size_t i = 0; i < total_steps; ++i)
        {
            std::vector<double> pt(BRANCHN_N * n_j, 0.0);

            // 填入分支2（左臂）
            for (int j = 0; j < n_j; ++j) pt[1 * n_j + j] = flat_joint_trajectory[i * 2 * n_j + j];

            // 填入分支3（右臂）
            for (int j = 0; j < n_j; ++j) pt[2 * n_j + j] = flat_joint_trajectory[i * 2 * n_j + n_j + j];

            // 其余分支保持不变
            for (int b = 0; b < BRANCHN_N; ++b)
            {
                if (b != 1 && b != 2)
                {
                    for (int j = 0; j < n_j; ++j) pt[b * n_j + j] = q_recv[b][j];
                }
            }

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

    goal_tf_mat_base_link2_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link2_0");
    goal_tf_mat_base_link3_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link3_0");

    ros::Rate loop_rate(200);  // 200Hz

    while (ros::ok())
    {
        // 使能并读取当前位姿
        if (control_flag == 0)
        {
            if (!isSimulation)
            {
                // 读取上电状态
                Motor_Rec_Func_ALL();

                robot_control::GetBaseLinkPose srv;
                for (int j = 0; j < 6; ++j)
                {
                    srv.request.joint_angles_branch1.push_back(q_recv[0][j]);
                    srv.request.joint_angles_branch4.push_back(q_recv[3][j]);
                }

                if (client.call(srv))
                {
                    ROS_INFO("Base link pose calculated: Position (x: %f, y: %f, z: %f), Orientation (x: %f, y: %f, z: %f, w: %f)", srv.response.base_link_pose.position.x, srv.response.base_link_pose.position.y, srv.response.base_link_pose.position.z, srv.response.base_link_pose.orientation.x,
                             srv.response.base_link_pose.orientation.y, srv.response.base_link_pose.orientation.z, srv.response.base_link_pose.orientation.w);

                    float_base_pub.publish(srv.response.base_link_pose);
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

        // 单臂操作-运动到初始位姿
        else if (control_flag == 1)
        {
            if (start_interp)
            {
                q_temp.resize(BRANCHN_N, std::vector<double>(MOTOR_BRANCHN_N, 0.0));
                q_temp = q_recv;  // 保存当前位置作为插值起点
                interp_step = 0;
                start_interp = false;
            }

            const int total_steps = 600;  // 30秒，200Hz
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
                gripper_command.data = {0.0, 1.0, 1.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;
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
                // 延时
                ros::Duration(1.0).sleep();  // 延时1秒，确保夹爪运动完成
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

        // 单臂操作-运动到初始位姿
        else if (control_flag == 888)
        {
            if (!planning_requested)
            {
                robot_planning::PlanPathHome srv;
                std::vector<double> init_floating_base = float_base_position;
                std::vector<double> init_joint_angles;
                init_joint_angles.insert(init_joint_angles.end(), q_recv[1].begin(), q_recv[1].begin() + 6);  // 左臂6个
                init_joint_angles.insert(init_joint_angles.end(), q_recv[2].begin(), q_recv[2].begin() + 6);  // 右臂6个
                std::vector<double> gold_floating_base = {0, 0, 0.45, 0, 0.7071, 0, 0.7071};
                std::vector<double> gold_joint_angles = {-1.18542, -1.84009, -1.90918, 1.5428, 1.18639, -2.28165, 1.20091, -1.84009, -1.90918, -1.54403, 1.20184, -0.859485};

                srv.request.init_floating_base = init_floating_base;
                srv.request.init_joint_angles = init_joint_angles;
                srv.request.gold_floating_base = gold_floating_base;
                srv.request.gold_joint_angles = gold_joint_angles;

                if (planning_client_home.call(srv))
                {
                    if (srv.response.success)
                    {
                        ROS_INFO("Planning request sent and completed successfully");
                        planning_requested = true;
                    }
                    else
                    {
                        ROS_ERROR("Planning request failed: %s", srv.response.message.c_str());
                        control_flag = 0;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call plan_path_home service");
                    control_flag = 0;
                }
            }
            else if (!planning_completed)
            {
                std::string robot_planning_path = ros::package::getPath("robot_planning");
                std::string planning_result_path = robot_planning_path + "/config/planning_result_home.yaml";
                std::ifstream file(planning_result_path);
                if (file.good())
                {
                    YAML::Node config = YAML::LoadFile(planning_result_path);

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

        // 单臂操作-运动到抓取准备位姿
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

                    robot_control::GetBaseLinkPose srv;
                    for (int j = 0; j < 6; ++j)
                    {
                        srv.request.joint_angles_branch1.push_back(planned_joint_trajectory[trajectory_index][j]);
                        srv.request.joint_angles_branch4.push_back(planned_joint_trajectory[trajectory_index][18 + j]);
                    }

                    if (client.call(srv))
                    {
                        ROS_INFO("Base link pose calculated: Position (x: %f, y: %f, z: %f), Orientation (x: %f, y: %f, z: %f, w: %f)", srv.response.base_link_pose.position.x, srv.response.base_link_pose.position.y, srv.response.base_link_pose.position.z, srv.response.base_link_pose.orientation.x,
                                 srv.response.base_link_pose.orientation.y, srv.response.base_link_pose.orientation.z, srv.response.base_link_pose.orientation.w);

                        float_base_pub.publish(srv.response.base_link_pose);
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
                control_flag = 11;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        // 单臂操作-分支2完成抓取
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

                // 统一调用服务
                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
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
                Eigen::Vector3d position_flan2 = tf_mat_link2_0_flan2.block<3, 1>(0, 3);  // 提取矩阵中的位置（x, y, z）

                Eigen::Quaterniond quat_flan2(tf_mat_link2_0_flan2.block<3, 3>(0, 0));  // 提取旋转部分并构造四元数
                Eigen::Vector4d quaternion_flan2 = quat_flan2.coeffs();                 // 获取四元数 [qx, qy, qz, qw]

                // 将位姿信息存储到 std::vector<double> 中
                std::vector<double> start_pose = {
                    position_flan2.x(),   position_flan2.y(),   position_flan2.z(),                         // [x, y, z]
                    quaternion_flan2.x(), quaternion_flan2.y(), quaternion_flan2.z(), quaternion_flan2.w()  // [qx, qy, qz, qw]
                };

                // // RRT
                // q_goal_rrt.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                // // 打印目标关节角度
                // std::cout << "RRT目标关节角度: ";
                // for (const auto &angle : q_goal_rrt)
                // {
                //     std::cout << angle << " ";
                // }
                // std::cout << std::endl;

                std::vector<double> traj2;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 6}, start_pose, goal_pose, plantime, traj2);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch2Only(traj2, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
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
                gripper_command.data = {0.0, 0.0, 1.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;
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
                ros::Duration(1.0).sleep();  // 延时1秒，确保夹爪运动完成
            }
        }

        // 单臂操作-分支2拿出物体（RRT）
        else if (control_flag == 301)
        {
            if (!planning_requested)
            {
                robot_rrt::RRTPlanPath srv;
                q_init_rrt.assign(q_recv[1].begin(), q_recv[1].begin() + 6);

                std::cout << "RRT初始关节角度: ";
                for (const auto &angle : q_init_rrt) std::cout << angle << " ";
                std::cout << std::endl;

                q_goal_rrt = {-1.3448, -1.81108, -2.4329, 3.1416, 0.23424, -1.31166};

                for (int i = 0; i < 6; ++i)
                {
                    srv.request.start[i] = q_init_rrt[i];
                    srv.request.goal[i] = q_goal_rrt[i];
                }

                if (rrt_plan_client.call(srv))
                {
                    if (srv.response.success)
                    {
                        ROS_INFO_STREAM("Planning succeeded: " << srv.response.message);

                        // 读取 YAML 文件
                        std::string yaml_path = ros::package::getPath("robot_rrt") + "/config/planned_path.yaml";
                        YAML::Node path_yaml = YAML::LoadFile(yaml_path);

                        std::vector<std::vector<double>> trajectory;
                        for (const auto &node : path_yaml) trajectory.push_back(node.as<std::vector<double>>());

                        // 保存原始插值轨迹
                        planned_joint_trajectory.clear();
                        const int steps_per_segment = 600;
                        for (size_t seg = 0; seg + 1 < trajectory.size(); ++seg)
                        {
                            const auto &q_start = trajectory[seg];
                            const auto &q_end = trajectory[seg + 1];

                            for (int step = 0; step <= steps_per_segment; ++step)
                            {
                                double ratio = static_cast<double>(step) / steps_per_segment;
                                std::vector<double> q_interp(6);
                                for (int j = 0; j < 6; ++j) q_interp[j] = q_start[j] + ratio * (q_end[j] - q_start[j]);
                                planned_joint_trajectory.push_back(q_interp);
                            }
                        }

                        // // 打印插值轨迹
                        // std::cout << "插值轨迹点数: " << planned_joint_trajectory.size() << std::endl;
                        // for (const auto &point : planned_joint_trajectory)
                        // {
                        //     std::cout << "插值点: ";
                        //     for (const auto &angle : point) std::cout << angle << " ";
                        //     std::cout << std::endl;
                        // }
                        // // 规划完成，设置标志

                        planning_requested = true;
                        trajectory_index = 0;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Planning failed: " << srv.response.message);
                        control_flag = 0;
                        return 0;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service rrt_plan_path.");
                    control_flag = 0;
                    return 0;
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                for (int j = 0; j < 6; ++j) q_send[1][j] = planned_joint_trajectory[trajectory_index][j];

                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj) q_recv[1][motorj] = q_send[1][motorj];
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
                ROS_INFO("RRT trajectory execution completed.");
                control_flag = 0;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;

                // 可选：发布夹爪指令或状态
                std_msgs::Float64MultiArray gripper_command;
                gripper_command.data = {0.0, 0.0, 1.0, 0.0};  // 示例：张开
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[1][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;

                std_msgs::Float64MultiArray motor_state;
                motor_state.data.resize(BRANCHN_N * MOTOR_BRANCHN_N);
                for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N; ++motorj) motor_state.data[branchi * MOTOR_BRANCHN_N + motorj] = q_recv[branchi][motorj];
                motor_state_pub.publish(motor_state);
            }
        }

        // 单臂操作-分支2拿出物体-1
        else if (control_flag == 4)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 4 received" << std::endl;

                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
                }

                Eigen::Matrix4d tf_mat_link2_0_flan2_goal = tf_mat_link2_0_flan2;

                Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
                translation(0, 3) = -0.10;
                // translation(2, 3) = -0.115;

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

                std::vector<double> traj2;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 6}, start_pose, goal_pose, plantime, traj2);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch2Only(traj2, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);

                if (isSimulation)
                {
                    // 统一调用服务
                    Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                    if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                    {
                        control_flag = 0;
                        return 0;
                    }

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

                    // tf_mat_flan2_grasp_object
                    Eigen::Affine3d tf_flan2_grasp_object = Eigen::Affine3d::Identity();
                    tf_flan2_grasp_object.translate(Eigen::Vector3d(-0.015, 0, 0.08));
                    tf_flan2_grasp_object.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
                    Eigen::Matrix4d tf_mat_flan2_grasp_object = tf_flan2_grasp_object.matrix();

                    // 计算grasp_object_pose
                    Eigen::Matrix4d tf_mat_world_grasp_object = tf_mat_world_base * goal_tf_mat_base_link2_0 * tf_mat_link2_0_flan2 * tf_mat_flan2_grasp_object;

                    Eigen::Vector3d position = tf_mat_world_grasp_object.block<3, 1>(0, 3);
                    Eigen::Matrix3d rotation = tf_mat_world_grasp_object.block<3, 3>(0, 0);
                    Eigen::Quaterniond quat(rotation.normalized());

                    std::vector<double> grasp_object_position(7);
                    grasp_object_position[0] = position.x();
                    grasp_object_position[1] = position.y();
                    grasp_object_position[2] = position.z();
                    grasp_object_position[3] = quat.x();
                    grasp_object_position[4] = quat.y();
                    grasp_object_position[5] = quat.z();
                    grasp_object_position[6] = quat.w();

                    // 发布grasp_object位置
                    geometry_msgs::Pose grasp_object_pose;
                    grasp_object_pose.position.x = grasp_object_position[0];
                    grasp_object_pose.position.y = grasp_object_position[1];
                    grasp_object_pose.position.z = grasp_object_position[2];
                    grasp_object_pose.orientation.x = grasp_object_position[3];
                    grasp_object_pose.orientation.y = grasp_object_position[4];
                    grasp_object_pose.orientation.z = grasp_object_position[5];
                    grasp_object_pose.orientation.w = grasp_object_position[6];
                    grasp_object_pub.publish(grasp_object_pose);
                }
            }
            else
            {
                ROS_INFO("Trajectory execution completed");
                control_flag = 401;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        // 单臂操作-分支2拿出物体-2
        else if (control_flag == 401)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 4 received" << std::endl;

                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
                }

                Eigen::Matrix4d tf_mat_link2_0_flan2_goal = tf_mat_link2_0_flan2;

                Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
                translation(2, 3) = -0.15;

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

                std::vector<double> traj2;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 6}, start_pose, goal_pose, plantime, traj2);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch2Only(traj2, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
                if (isSimulation)
                {
                    // 统一调用服务
                    Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                    if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                    {
                        control_flag = 0;
                        return 0;
                    }

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

                    // tf_mat_flan2_grasp_object
                    Eigen::Affine3d tf_flan2_grasp_object = Eigen::Affine3d::Identity();
                    tf_flan2_grasp_object.translate(Eigen::Vector3d(-0.015, 0, 0.08));
                    tf_flan2_grasp_object.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
                    Eigen::Matrix4d tf_mat_flan2_grasp_object = tf_flan2_grasp_object.matrix();

                    // 计算grasp_object_pose
                    Eigen::Matrix4d tf_mat_world_grasp_object = tf_mat_world_base * goal_tf_mat_base_link2_0 * tf_mat_link2_0_flan2 * tf_mat_flan2_grasp_object;

                    Eigen::Vector3d position = tf_mat_world_grasp_object.block<3, 1>(0, 3);
                    Eigen::Matrix3d rotation = tf_mat_world_grasp_object.block<3, 3>(0, 0);
                    Eigen::Quaterniond quat(rotation.normalized());

                    std::vector<double> grasp_object_position(7);
                    grasp_object_position[0] = position.x();
                    grasp_object_position[1] = position.y();
                    grasp_object_position[2] = position.z();
                    grasp_object_position[3] = quat.x();
                    grasp_object_position[4] = quat.y();
                    grasp_object_position[5] = quat.z();
                    grasp_object_position[6] = quat.w();

                    // 发布grasp_object位置
                    geometry_msgs::Pose grasp_object_pose;
                    grasp_object_pose.position.x = grasp_object_position[0];
                    grasp_object_pose.position.y = grasp_object_position[1];
                    grasp_object_pose.position.z = grasp_object_position[2];
                    grasp_object_pose.orientation.x = grasp_object_position[3];
                    grasp_object_pose.orientation.y = grasp_object_position[4];
                    grasp_object_pose.orientation.z = grasp_object_position[5];
                    grasp_object_pose.orientation.w = grasp_object_position[6];
                    grasp_object_pub.publish(grasp_object_pose);
                }
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

        // 单臂操作-分支2运动到交接位姿
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
                Eigen::Matrix4d tf_mat_link2_0_flan2_goal = tf_mat_base_link2_0.inverse() * tf_mat_base_obj_1;

                // 统一调用服务
                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
                }

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

                std::vector<double> traj2;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 6}, start_pose, goal_pose, plantime, traj2);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch2Only(traj2, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
                if (isSimulation)
                {
                    // 统一调用服务
                    Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                    if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                    {
                        control_flag = 0;
                        return 0;
                    }

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

                    // tf_mat_flan2_grasp_object
                    Eigen::Affine3d tf_flan2_grasp_object = Eigen::Affine3d::Identity();
                    tf_flan2_grasp_object.translate(Eigen::Vector3d(-0.015, 0, 0.08));
                    tf_flan2_grasp_object.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
                    Eigen::Matrix4d tf_mat_flan2_grasp_object = tf_flan2_grasp_object.matrix();

                    // 计算grasp_object_pose
                    Eigen::Matrix4d tf_mat_world_grasp_object = tf_mat_world_base * goal_tf_mat_base_link2_0 * tf_mat_link2_0_flan2 * tf_mat_flan2_grasp_object;

                    Eigen::Vector3d position = tf_mat_world_grasp_object.block<3, 1>(0, 3);
                    Eigen::Matrix3d rotation = tf_mat_world_grasp_object.block<3, 3>(0, 0);
                    Eigen::Quaterniond quat(rotation.normalized());

                    std::vector<double> grasp_object_position(7);
                    grasp_object_position[0] = position.x();
                    grasp_object_position[1] = position.y();
                    grasp_object_position[2] = position.z();
                    grasp_object_position[3] = quat.x();
                    grasp_object_position[4] = quat.y();
                    grasp_object_position[5] = quat.z();
                    grasp_object_position[6] = quat.w();

                    // 发布grasp_object位置
                    geometry_msgs::Pose grasp_object_pose;
                    grasp_object_pose.position.x = grasp_object_position[0];
                    grasp_object_pose.position.y = grasp_object_position[1];
                    grasp_object_pose.position.z = grasp_object_position[2];
                    grasp_object_pose.orientation.x = grasp_object_position[3];
                    grasp_object_pose.orientation.y = grasp_object_position[4];
                    grasp_object_pose.orientation.z = grasp_object_position[5];
                    grasp_object_pose.orientation.w = grasp_object_position[6];
                    grasp_object_pub.publish(grasp_object_pose);
                }
            }
            else
            {
                ROS_INFO("Trajectory execution completed");
                control_flag = 6;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
                if (isSimulation)
                {
                    // 统一调用服务
                    Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                    if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                    {
                        control_flag = 0;
                        return 0;
                    }

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

                    // tf_mat_flan2_grasp_object
                    Eigen::Affine3d tf_flan2_grasp_object = Eigen::Affine3d::Identity();
                    tf_flan2_grasp_object.translate(Eigen::Vector3d(-0.015, 0, 0.08));
                    tf_flan2_grasp_object.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
                    Eigen::Matrix4d tf_mat_flan2_grasp_object = tf_flan2_grasp_object.matrix();

                    // 计算grasp_object_pose
                    Eigen::Matrix4d tf_mat_world_grasp_object = tf_mat_world_base * goal_tf_mat_base_link2_0 * tf_mat_link2_0_flan2 * tf_mat_flan2_grasp_object;

                    // 打印tf_mat_world_grasp_object
                    std::cout << "Transform Matrix (world -> grasp_object):\n" << tf_mat_world_grasp_object << "\n" << std::endl;

                    Eigen::Vector3d position = tf_mat_world_grasp_object.block<3, 1>(0, 3);
                    Eigen::Matrix3d rotation = tf_mat_world_grasp_object.block<3, 3>(0, 0);
                    Eigen::Quaterniond quat(rotation.normalized());

                    std::vector<double> grasp_object_position(7);
                    grasp_object_position[0] = position.x();
                    grasp_object_position[1] = position.y();
                    grasp_object_position[2] = position.z();
                    grasp_object_position[3] = quat.x();
                    grasp_object_position[4] = quat.y();
                    grasp_object_position[5] = quat.z();
                    grasp_object_position[6] = quat.w();

                    // 发布grasp_object位置
                    geometry_msgs::Pose grasp_object_pose;
                    grasp_object_pose.position.x = grasp_object_position[0];
                    grasp_object_pose.position.y = grasp_object_position[1];
                    grasp_object_pose.position.z = grasp_object_position[2];
                    grasp_object_pose.orientation.x = grasp_object_position[3];
                    grasp_object_pose.orientation.y = grasp_object_position[4];
                    grasp_object_pose.orientation.z = grasp_object_position[5];
                    grasp_object_pose.orientation.w = grasp_object_position[6];
                    grasp_object_pub.publish(grasp_object_pose);
                }
            }
        }

        // 单臂操作-分支3运动到交接位姿完成交接
        else if (control_flag == 6)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 6 received" << std::endl;

                // 统一调用服务
                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
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

                std::vector<double> traj3;
                try
                {
                    planBranch(1, {q_recv[2].begin(), q_recv[2].begin() + 6}, start_pose, goal_pose, plantime, traj3);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch3Only(traj3, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
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
                gripper_command.data = {0.0, 0.0, 0.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;
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
                ros::Duration(1.0).sleep();  // 延时1秒，确保夹爪运动完成
                                             // 发布夹爪指令
                gripper_command.data = {0.0, 1.0, 0.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;
                // 发布电机位置状态
                motor_state.data.resize(BRANCHN_N * MOTOR_BRANCHN_N);
                for (int branchi = 0; branchi < BRANCHN_N; branchi++)
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N; motorj++)
                    {
                        motor_state.data[branchi * MOTOR_BRANCHN_N + motorj] = q_recv[branchi][motorj];
                    }
                }
                motor_state_pub.publish(motor_state);
                ros::Duration(1.0).sleep();  // 延时1秒，确保夹爪运动完成
            }
        }

        // 单臂操作-分支2离开交接位姿
        else if (control_flag == 7)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 7 received" << std::endl;

                // 统一调用服务
                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
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

                std::vector<double> traj2;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 6}, start_pose, goal_pose, plantime, traj2);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch2Only(traj2, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
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

        // 单臂操作-分支3运动到放置准备位姿
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

                // 统一调用服务
                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
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

                std::vector<double> traj3;
                try
                {
                    planBranch(2, {q_recv[2].begin(), q_recv[2].begin() + 6}, start_pose, goal_pose, plantime, traj3);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch3Only(traj3, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
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

        // 单臂操作-分支3完成放置
        else if (control_flag == 9)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 9 received" << std::endl;

                // 统一调用服务
                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
                }

                Eigen::Matrix4d tf_mat_link3_0_flan3_goal = tf_mat_link3_0_flan3;

                // 构造平移矩阵
                Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
                translation(0, 3) = -0.05;

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

                std::vector<double> traj3;
                try
                {
                    planBranch(2, {q_recv[2].begin(), q_recv[2].begin() + 6}, start_pose, goal_pose, plantime, traj3);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch3Only(traj3, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
            }
            else
            {
                ROS_INFO("Trajectory execution completed");
                control_flag = 901;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;

                // 发布夹爪指令
                std_msgs::Float64MultiArray gripper_command;
                gripper_command.data = {0.0, 1.0, 1.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;
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
                ros::Duration(1.0).sleep();  // 延时1秒，确保夹爪运动完成

                if (isSimulation)
                {
                    // 统一调用服务
                    Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                    if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                    {
                        control_flag = 0;
                        return 0;
                    }

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

                    // tf_mat_flan3_grasp_object
                    Eigen::Affine3d tf_flan3_grasp_object = Eigen::Affine3d::Identity();
                    tf_flan3_grasp_object.translate(Eigen::Vector3d(-0.015, 0, 0.08));
                    tf_flan3_grasp_object.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
                    Eigen::Matrix4d tf_mat_flan3_grasp_object = tf_flan3_grasp_object.matrix();

                    // 计算grasp_object_pose
                    Eigen::Matrix4d tf_mat_world_grasp_object = tf_mat_world_base * goal_tf_mat_base_link3_0 * tf_mat_link3_0_flan3 * tf_mat_flan3_grasp_object;

                    // 打印tf_mat_world_grasp_object
                    std::cout << "Transform Matrix (world -> grasp_object):\n" << tf_mat_world_grasp_object << "\n" << std::endl;

                    Eigen::Vector3d position = tf_mat_world_grasp_object.block<3, 1>(0, 3);
                    Eigen::Matrix3d rotation = tf_mat_world_grasp_object.block<3, 3>(0, 0);
                    Eigen::Quaterniond quat(rotation.normalized());

                    std::vector<double> grasp_object_position(7);
                    grasp_object_position[0] = position.x();
                    grasp_object_position[1] = position.y();
                    grasp_object_position[2] = position.z();
                    grasp_object_position[3] = quat.x();
                    grasp_object_position[4] = quat.y();
                    grasp_object_position[5] = quat.z();
                    grasp_object_position[6] = quat.w();

                    // 发布grasp_object位置
                    geometry_msgs::Pose grasp_object_pose;
                    grasp_object_pose.position.x = grasp_object_position[0];
                    grasp_object_pose.position.y = grasp_object_position[1];
                    grasp_object_pose.position.z = grasp_object_position[2];
                    grasp_object_pose.orientation.x = grasp_object_position[3];
                    grasp_object_pose.orientation.y = grasp_object_position[4];
                    grasp_object_pose.orientation.z = grasp_object_position[5];
                    grasp_object_pose.orientation.w = grasp_object_position[6];
                    grasp_object_pub.publish(grasp_object_pose);
                }
            }
        }

        // 导纳控制
        else if (control_flag == 21)
        {
            // 0. 参数
            const Eigen::Matrix<double, 6, 1> M = (Eigen::Matrix<double, 6, 1>() << 2.0, 2.0, 2.0, 0.2, 0.2, 0.2).finished();
            const Eigen::Matrix<double, 6, 1> B = (Eigen::Matrix<double, 6, 1>() << 40.0, 40.0, 40.0, 3.0, 3.0, 3.0).finished();
            const Eigen::Matrix<double, 6, 1> K = (Eigen::Matrix<double, 6, 1>() << 100.0, 100.0, 100.0, 5.0, 5.0, 5.0).finished();

            const double MAX_TRANSL = 0.02;                 // m
            const double MAX_ANGLE = 8.0 * M_PI / 180.0;    // rad
            const double MAX_POSVEL = 0.30;                 // m/s
            const double MAX_ANGVEL = 45.0 * M_PI / 180.0;  // rad/s

            // 1. 锁定 T_ref
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

            if (!log_initialized)
            {
                admittance_log.open(csv_file_path, std::ios::out);
                admittance_log << "time,Fx,Fy,Fz,Tx,Ty,Tz,dx,dy,dz,dRx,dRy,dRz\n";
                log_initialized = true;
            }

            // 2. 导纳内部状态
            static Eigen::Matrix<double, 6, 1> x_int = Eigen::Matrix<double, 6, 1>::Zero();    // 位移
            static Eigen::Matrix<double, 6, 1> x_int_d = Eigen::Matrix<double, 6, 1>::Zero();  // 速度
            static ros::Time last_t = ros::Time::now();

            double dt = (ros::Time::now() - last_t).toSec();
            last_t = ros::Time::now();
            if (dt <= 0.)
                dt = 1e-3;

            // 3. 力‑扭矩误差
            Eigen::Matrix<double, 6, 1> F_des = Eigen::Matrix<double, 6, 1>::Zero();
            Eigen::Matrix<double, 6, 1> F_meas;
            F_meas << force_data_3.force.x, force_data_3.force.y, force_data_3.force.z, force_data_3.torque.x, force_data_3.torque.y, force_data_3.torque.z;
            Eigen::Matrix<double, 6, 1> F_err = F_des - F_meas;

            // 4. 导纳微分方程
            Eigen::Matrix<double, 6, 1> x_dd = (F_err - B.cwiseProduct(x_int_d) - K.cwiseProduct(x_int)).cwiseQuotient(M);

            x_int_d += x_dd * dt;

            /* 限速 */
            x_int_d.head<3>() = x_int_d.head<3>().cwiseMax(-MAX_POSVEL).cwiseMin(MAX_POSVEL);
            x_int_d.tail<3>() = x_int_d.tail<3>().cwiseMax(-MAX_ANGVEL).cwiseMin(MAX_ANGVEL);

            x_int += x_int_d * dt;

            /* 限幅 */
            x_int.head<3>() = x_int.head<3>().cwiseMax(-MAX_TRANSL).cwiseMin(MAX_TRANSL);
            x_int.tail<3>() = x_int.tail<3>().cwiseMax(-MAX_ANGLE).cwiseMin(MAX_ANGLE);

            // 5. 生成目标位姿
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

            /* ---------- 记录数据 ---------- */
            static ros::Time t_start = ros::Time::now();
            double t_now = (ros::Time::now() - t_start).toSec();

            admittance_log << t_now << "," << F_meas(0) << "," << F_meas(1) << "," << F_meas(2) << "," << F_meas(3) << "," << F_meas(4) << "," << F_meas(5) << "," << x_int(0) << "," << x_int(1) << "," << x_int(2) << "," << x_int(3) * 180.0 / M_PI << "," << x_int(4) * 180.0 / M_PI << ","
                           << x_int(5) * 180.0 / M_PI << "\n";

            // 6. IKFast
            std::vector<float> ee_pose12 = {(float)T_tgt(0, 0), (float)T_tgt(0, 1), (float)T_tgt(0, 2), (float)T_tgt(0, 3), (float)T_tgt(1, 0), (float)T_tgt(1, 1), (float)T_tgt(1, 2), (float)T_tgt(1, 3), (float)T_tgt(2, 0), (float)T_tgt(2, 1), (float)T_tgt(2, 2), (float)T_tgt(2, 3)};
            std::vector<float> ik_results = kin_arm3.inverse(ee_pose12);

            std::vector<float> q_cur_vec(6);
            for (int j = 0; j < 6; ++j) q_cur_vec[j] = q_recv[2][j];

            auto [valid_sols, best_sol] = findAllSolutions(ik_results, q_cur_vec, 6);

            // 7. 写入 q_send
            if (best_sol.empty())
            {
                ROS_WARN_THROTTLE(1.0, "IKFast 无有效解，保持当前位置");
                for (int j = 0; j < 6; ++j) q_send[2][j] = q_recv[2][j];
            }
            else
            {
                for (int j = 0; j < 6; ++j) q_send[2][j] = best_sol[j];
            }

            // 8. 下发 / 仿真
            if (!isSimulation)
                Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
            else
                for (int j = 0; j < 6; ++j) q_recv[2][j] = q_send[2][j];

            // 9. 发布 joint state
            std_msgs::Float64MultiArray motor_state;
            motor_state.data.resize(BRANCHN_N * MOTOR_BRANCHN_N);
            for (int b = 0; b < BRANCHN_N; ++b)
                for (int j = 0; j < MOTOR_BRANCHN_N; ++j) motor_state.data[b * MOTOR_BRANCHN_N + j] = q_recv[b][j];
            motor_state_pub.publish(motor_state);
        }

        // 单臂操作-分支3移动到最终位姿
        else if (control_flag == 901)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 901 received" << std::endl;

                // 统一调用服务
                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
                }

                Eigen::Matrix4d tf_mat_link3_0_flan3_goal = tf_mat_link3_0_flan3;

                // 构造平移矩阵
                Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
                translation(0, 3) = 0.05;

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

                std::vector<double> traj3;
                try
                {
                    planBranch(2, {q_recv[2].begin(), q_recv[2].begin() + 6}, start_pose, goal_pose, plantime, traj3);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch3Only(traj3, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
            }
            else
            {
                ROS_INFO("Trajectory execution completed");
                control_flag = 10;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        // 双臂操作-分支2、3移动到准备位姿
        else if (control_flag == 10)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 10 received" << std::endl;

                // ==== STEP 1: 加载必要的变换矩阵 ====
                Eigen::Matrix4d tf_world_cube_r = loadTransformFromYAML(common_tf_path, "tf_mat_world_cube_r");
                Eigen::Matrix4d tf_world_cube_l = loadTransformFromYAML(common_tf_path, "tf_mat_world_cube_l");
                Eigen::Matrix4d tf_base_link3_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link3_0");
                Eigen::Matrix4d tf_base_link2_0 = loadTransformFromYAML(common_tf_path, "tf_mat_base_link2_0");

                // ==== STEP 2: 构造 world → base 变换 ====
                Eigen::Vector3d trans_base(float_base_position[0], float_base_position[1], float_base_position[2]);
                Eigen::Quaterniond quat_base(float_base_position[6], float_base_position[3], float_base_position[4], float_base_position[5]);
                Eigen::Matrix3d rot_base = quat_base.normalized().toRotationMatrix();
                Eigen::Matrix4d tf_world_base = Eigen::Matrix4d::Identity();
                tf_world_base.block<3, 3>(0, 0) = rot_base;
                tf_world_base.block<3, 1>(0, 3) = trans_base;

                // ==== STEP 3: 计算 link → cube 变换 ====
                Eigen::Matrix4d tf_link3_0_cube_r = tf_base_link3_0.inverse() * tf_world_base.inverse() * tf_world_cube_r;
                Eigen::Matrix4d tf_link2_0_cube_l = tf_base_link2_0.inverse() * tf_world_base.inverse() * tf_world_cube_l;

                Eigen::Matrix4d T2_goal, T3_goal;
                /* ---- 目标位姿 (+Z) ---- */
                Eigen::Matrix4d TZp = Eigen::Matrix4d::Identity();
                TZp(2, 3) = -0.03;
                T2_goal = tf_link2_0_cube_l * TZp;
                T3_goal = tf_link3_0_cube_r * TZp;

                // ==== STEP 4: 获取当前末端位姿 ====
                Eigen::Matrix4d tf_link2_0_flan2, tf_link3_0_flan3;
                if (!getCurrentEEPose(tf_link2_0_flan2, tf_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
                }

                auto matrixToPoseVec = [](const Eigen::Matrix4d &tf) -> std::vector<double>
                {
                    Eigen::Vector3d position = tf.block<3, 1>(0, 3);
                    Eigen::Quaterniond quat(tf.block<3, 3>(0, 0));
                    Eigen::Vector4d q = quat.coeffs();  // [x, y, z, w]
                    return {position.x(), position.y(), position.z(), q.x(), q.y(), q.z(), q.w()};
                };

                std::vector<double> goal_pose_r = matrixToPoseVec(T3_goal);
                std::vector<double> goal_pose_l = matrixToPoseVec(T2_goal);
                std::vector<double> start_pose_r = matrixToPoseVec(tf_link3_0_flan3);
                std::vector<double> start_pose_l = matrixToPoseVec(tf_link2_0_flan2);

                // ==== STEP 5: 调用规划函数 ====
                std::vector<double> traj2, traj3;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 6}, start_pose_l, goal_pose_l, 5.0, traj2);  // 左臂
                    planBranch(2, {q_recv[2].begin(), q_recv[2].begin() + 6}, start_pose_r, goal_pose_r, 5.0, traj3);  // 右臂
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }

                // ==== STEP 6: 合并两个分支的轨迹 ====
                mergeTraj(traj2, traj3, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
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

        // 双臂操作-分支2、3移动到抓取位姿
        else if (control_flag == 11)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 11 received" << std::endl;
                ROS_INFO("=== flag 11 : Z +0.03 ===");

                /* ---- 当前位姿 ---- */
                Eigen::Matrix4d T2_cur, T3_cur;
                if (!getCurrentEEPose(T2_cur, T3_cur))
                {
                    control_flag = 0;
                    return 0;
                }

                /* ---- 目标位姿 (+Z) ---- */
                Eigen::Matrix4d TZp = Eigen::Matrix4d::Identity();
                TZp(2, 3) = 0.05;
                Eigen::Matrix4d T2_goal = T2_cur * TZp;
                Eigen::Matrix4d T3_goal = T3_cur * TZp;

                /* ---- 规划两分支 ---- */
                std::vector<double> traj2, traj3;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 5}, matToPose(T2_cur), matToPose(T2_goal), plantime, traj2);
                    planBranch(2, {q_recv[2].begin(), q_recv[2].begin() + 5}, matToPose(T3_cur), matToPose(T3_goal), plantime, traj3);
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
                control_flag = 12;
                planning_requested = planning_completed = false;
                trajectory_index = 0;

                // 发布夹爪指令
                std_msgs::Float64MultiArray gripper_command;
                gripper_command.data = {0.0, 0.0, 0.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;
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
                ros::Duration(1.0).sleep();  // 延时1秒，确保夹爪运动完成
            }
        }

        // 双臂操作-物体移动到准备位姿
        else if (control_flag == 12)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 12 received" << std::endl;
                // 读取YAML中的世界→cube_m，world→cube_m 变换
                Eigen::Matrix4d tf_mat_world_cube_m = loadTransformFromYAML(common_tf_path, "tf_mat_world_cube_m");
                Eigen::Matrix4d tf_mat_cube_m_r = loadTransformFromYAML(common_tf_path, "tf_mat_cube_m_r");
                Eigen::Matrix4d tf_mat_cube_m_l = loadTransformFromYAML(common_tf_path, "tf_mat_cube_m_l");

                Eigen::Matrix4d tf_mat_world_cube_m_init = tf_mat_world_cube_m;
                // 构造沿Y轴正向平移0.12米的齐次变换
                Eigen::Matrix4d translation_mat = Eigen::Matrix4d::Identity();
                translation_mat(1, 3) = 0.12;

                // 得到目标变换
                Eigen::Matrix4d tf_mat_world_cube_m_goal = tf_mat_world_cube_m_init * translation_mat;

                robot_planning::PlanDualArmPath srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.tf_mat_world_cube_m_init = eigenToTransformMsg(tf_mat_world_cube_m_init);
                srv.request.tf_mat_world_cube_m_goal = eigenToTransformMsg(tf_mat_world_cube_m_goal);
                srv.request.tf_mat_cube_m_l = eigenToTransformMsg(tf_mat_cube_m_l);
                srv.request.tf_mat_cube_m_r = eigenToTransformMsg(tf_mat_cube_m_r);
                srv.request.num_interpolations = 1000;  // 或任意你想设置的插值点数量
                if (plan_dual_arm_path_client.call(srv))
                {
                    std::cout << "Service call successful" << std::endl;
                }
                else
                {
                    ROS_ERROR("Failed to call service planDualArmPath");
                }

                mergeTrajFromFlat(srv.response.joint_trajectory, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
            }
            else
            {
                control_flag = 121;
                planning_requested = planning_completed = false;
                trajectory_index = 0;
            }
        }

        else if (control_flag == 121)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 121 received" << std::endl;
                // 读取YAML中的世界→cube_m，world→cube_m 变换
                Eigen::Matrix4d tf_mat_world_cube_m = loadTransformFromYAML(common_tf_path, "tf_mat_world_cube_m");
                Eigen::Matrix4d tf_mat_cube_m_r = loadTransformFromYAML(common_tf_path, "tf_mat_cube_m_r");
                Eigen::Matrix4d tf_mat_cube_m_l = loadTransformFromYAML(common_tf_path, "tf_mat_cube_m_l");

                Eigen::Matrix4d TZp = Eigen::Matrix4d::Identity();
                TZp(1, 3) = 0.12;
                Eigen::Matrix4d tf_mat_world_cube_m_init = tf_mat_world_cube_m * TZp;

                // 构造沿X轴正向平移0.015米的齐次变换
                Eigen::Matrix4d translation_mat = Eigen::Matrix4d::Identity();
                translation_mat(0, 3) = 0.015;

                // 得到目标变换
                Eigen::Matrix4d tf_mat_world_cube_m_goal = tf_mat_world_cube_m_init * translation_mat;

                robot_planning::PlanDualArmPath srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.tf_mat_world_cube_m_init = eigenToTransformMsg(tf_mat_world_cube_m_init);
                srv.request.tf_mat_world_cube_m_goal = eigenToTransformMsg(tf_mat_world_cube_m_goal);
                srv.request.tf_mat_cube_m_l = eigenToTransformMsg(tf_mat_cube_m_l);
                srv.request.tf_mat_cube_m_r = eigenToTransformMsg(tf_mat_cube_m_r);
                srv.request.num_interpolations = 1000;  // 或任意你想设置的插值点数量
                if (plan_dual_arm_path_client.call(srv))
                {
                    std::cout << "Service call successful" << std::endl;
                }
                else
                {
                    ROS_ERROR("Failed to call service planDualArmPath");
                }

                mergeTrajFromFlat(srv.response.joint_trajectory, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
            }
            else
            {
                control_flag = 13;
                planning_requested = planning_completed = false;
                trajectory_index = 0;
            }
        }

        // 双臂操作-机身移动到目标位姿
        else if (control_flag == 13)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 13 received" << std::endl;
                robot_planning::PlanPathHome srv;
                std::vector<double> init_floating_base = float_base_position;
                std::vector<double> init_joint_angles;
                init_joint_angles.insert(init_joint_angles.end(), q_recv[1].begin(), q_recv[1].begin() + 6);  // 左臂6个
                init_joint_angles.insert(init_joint_angles.end(), q_recv[2].begin(), q_recv[2].begin() + 6);  // 右臂6个

                // gold_floating_base 相对于 init_floating_base 沿 y 轴移动 0.15 米
                std::vector<double> gold_floating_base = init_floating_base;
                if (gold_floating_base.size() >= 2)
                {
                    gold_floating_base[1] += 0.1;  // y 轴坐标增加 0.15
                }
                else
                {
                    ROS_WARN_STREAM("init_floating_base size is too small to modify Y component.");
                }

                std::vector<double> gold_joint_angles = init_joint_angles;
                srv.request.init_floating_base = init_floating_base;
                srv.request.init_joint_angles = init_joint_angles;
                srv.request.gold_floating_base = gold_floating_base;
                srv.request.gold_joint_angles = gold_joint_angles;

                if (planning_client_home.call(srv))
                {
                    if (srv.response.success)
                    {
                        ROS_INFO("Planning request sent and completed successfully");
                        planning_requested = true;
                    }
                    else
                    {
                        ROS_ERROR("Planning request failed: %s", srv.response.message.c_str());
                        control_flag = 0;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call plan_path_home service");
                    control_flag = 0;
                }
            }
            else if (!planning_completed)
            {
                std::string robot_planning_path = ros::package::getPath("robot_planning");
                std::string planning_result_path = robot_planning_path + "/config/planning_result_home.yaml";
                std::ifstream file(planning_result_path);
                if (file.good())
                {
                    YAML::Node config = YAML::LoadFile(planning_result_path);

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

                    robot_control::GetBaseLinkPose srv;
                    for (int j = 0; j < 6; ++j)
                    {
                        srv.request.joint_angles_branch1.push_back(planned_joint_trajectory[trajectory_index][j]);
                        srv.request.joint_angles_branch4.push_back(planned_joint_trajectory[trajectory_index][18 + j]);
                    }

                    if (client.call(srv))
                    {
                        ROS_INFO("Base link pose calculated: Position (x: %f, y: %f, z: %f), Orientation (x: %f, y: %f, z: %f, w: %f)", srv.response.base_link_pose.position.x, srv.response.base_link_pose.position.y, srv.response.base_link_pose.position.z, srv.response.base_link_pose.orientation.x,
                                 srv.response.base_link_pose.orientation.y, srv.response.base_link_pose.orientation.z, srv.response.base_link_pose.orientation.w);

                        float_base_pub.publish(srv.response.base_link_pose);

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
                control_flag = 14;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        // 双臂操作-分支2、3移动到目标位姿
        else if (control_flag == 14)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 14 received" << std::endl;
                // 读取YAML中的世界→cube_m，world→cube_m 变换
                Eigen::Matrix4d tf_mat_world_cube_m = loadTransformFromYAML(common_tf_path, "tf_mat_world_cube_m");
                Eigen::Matrix4d tf_mat_cube_m_r = loadTransformFromYAML(common_tf_path, "tf_mat_cube_m_r");
                Eigen::Matrix4d tf_mat_cube_m_l = loadTransformFromYAML(common_tf_path, "tf_mat_cube_m_l");

                Eigen::Matrix4d TZp = Eigen::Matrix4d::Identity();
                TZp(1, 3) = 0.12;
                TZp(0, 3) = 0.015;
                TZp(2, 3) = 0.1;
                Eigen::Matrix4d tf_mat_world_cube_m_init = tf_mat_world_cube_m * TZp;

                // 构造沿Y轴正向平移0.12米的齐次变换
                Eigen::Matrix4d translation_mat = Eigen::Matrix4d::Identity();
                translation_mat(1, 3) = -0.02;

                // 得到目标变换
                Eigen::Matrix4d tf_mat_world_cube_m_goal = tf_mat_world_cube_m_init * translation_mat;

                robot_planning::PlanDualArmPath srv;
                srv.request.float_base_pose = float_base_position;
                srv.request.branch2_joints.assign(q_recv[1].begin(), q_recv[1].begin() + 6);
                srv.request.branch3_joints.assign(q_recv[2].begin(), q_recv[2].begin() + 6);
                srv.request.tf_mat_world_cube_m_init = eigenToTransformMsg(tf_mat_world_cube_m_init);
                srv.request.tf_mat_world_cube_m_goal = eigenToTransformMsg(tf_mat_world_cube_m_goal);
                srv.request.tf_mat_cube_m_l = eigenToTransformMsg(tf_mat_cube_m_l);
                srv.request.tf_mat_cube_m_r = eigenToTransformMsg(tf_mat_cube_m_r);
                srv.request.num_interpolations = 1000;  // 或任意你想设置的插值点数量
                if (plan_dual_arm_path_client.call(srv))
                {
                    std::cout << "Service call successful" << std::endl;
                }
                else
                {
                    ROS_ERROR("Failed to call service planDualArmPath");
                }

                mergeTrajFromFlat(srv.response.joint_trajectory, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
            }
            else
            {
                control_flag = 15;
                planning_requested = planning_completed = false;
                trajectory_index = 0;

                // 发布夹爪指令
                std_msgs::Float64MultiArray gripper_command;
                gripper_command.data = {0.0, 1.0, 1.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;
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
                ros::Duration(1.0).sleep();  // 延时1秒，确保夹爪运动完成
            }
        }

        // 单臂操作-分支2离开交接位姿
        else if (control_flag == 15)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 7 received" << std::endl;

                // 统一调用服务
                Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
                if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
                {
                    control_flag = 0;
                    return 0;
                }

                Eigen::Matrix4d tf_mat_link2_0_flan2_goal = tf_mat_link2_0_flan2;

                // 构造平移矩阵
                Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
                translation(2, 3) = -0.05;

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

                std::vector<double> traj2;
                try
                {
                    planBranch(1, {q_recv[1].begin(), q_recv[1].begin() + 6}, start_pose, goal_pose, plantime, traj2);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("%s", e.what());
                    control_flag = 0;
                    return 0;
                }
                mergeBranch2Only(traj2, planned_joint_trajectory);
                planning_requested = true;
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                executeStep(trajectory_index++);
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

        else if (control_flag == 999)
        {
            if (!planning_requested)
            {
                robot_planning::PlanPathHome srv;
                std::vector<double> init_floating_base = float_base_position;
                std::vector<double> init_joint_angles;
                init_joint_angles.insert(init_joint_angles.end(), q_recv[1].begin(), q_recv[1].begin() + 6);  // 左臂6个
                init_joint_angles.insert(init_joint_angles.end(), q_recv[2].begin(), q_recv[2].begin() + 6);  // 右臂6个
                std::vector<double> gold_floating_base = {0, 0, 0.45, 0, 0.7071, 0, 0.7071};
                std::vector<double> gold_joint_angles = {-1.18542, -1.84009, -1.90918, 1.5428, 1.18639, -2.28165, 1.20091, -1.84009, -1.90918, -1.54403, 1.20184, -0.859485};

                srv.request.init_floating_base = init_floating_base;
                srv.request.init_joint_angles = init_joint_angles;
                srv.request.gold_floating_base = gold_floating_base;
                srv.request.gold_joint_angles = gold_joint_angles;

                if (planning_client_home.call(srv))
                {
                    if (srv.response.success)
                    {
                        ROS_INFO("Planning request sent and completed successfully");
                        planning_requested = true;
                    }
                    else
                    {
                        ROS_ERROR("Planning request failed: %s", srv.response.message.c_str());
                        control_flag = 0;
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call plan_path_home service");
                    control_flag = 0;
                }
            }
            else if (!planning_completed)
            {
                std::string robot_planning_path = ros::package::getPath("robot_planning");
                std::string planning_result_path = robot_planning_path + "/config/planning_result_home.yaml";
                std::ifstream file(planning_result_path);
                if (file.good())
                {
                    YAML::Node config = YAML::LoadFile(planning_result_path);

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

                    robot_control::GetBaseLinkPose srv;

                    for (int j = 0; j < 6; ++j)
                    {
                        srv.request.joint_angles_branch1.push_back(planned_joint_trajectory[trajectory_index][j]);
                        srv.request.joint_angles_branch4.push_back(planned_joint_trajectory[trajectory_index][18 + j]);
                    }

                    if (client.call(srv))
                    {
                        ROS_INFO("Base link pose calculated: Position (x: %f, y: %f, z: %f), Orientation (x: %f, y: %f, z: %f, w: %f)", srv.response.base_link_pose.position.x, srv.response.base_link_pose.position.y, srv.response.base_link_pose.position.z, srv.response.base_link_pose.orientation.x,
                                 srv.response.base_link_pose.orientation.y, srv.response.base_link_pose.orientation.z, srv.response.base_link_pose.orientation.w);

                        float_base_pub.publish(srv.response.base_link_pose);
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

        // MPC
        else if (control_flag == 666)
        {
            if (!planning_requested)
            {
                std::string log_path = ros::package::getPath("robot_mpc") + "/config/mpc_result_log.yaml";
                YAML::Node log = YAML::LoadFile(log_path);

                planned_joint_trajectory.clear();

                if (log["trajectory_log"])
                {
                    for (const auto &step : log["trajectory_log"])
                    {
                        if (step["q"])
                        {
                            std::vector<double> q = step["q"].as<std::vector<double>>();
                            planned_joint_trajectory.push_back(q);
                        }
                    }
                    std::cout << "Loaded " << planned_joint_trajectory.size() << " trajectory steps from log." << std::endl;

                    planning_requested = true;
                    trajectory_index = 0;
                }
                else
                {
                    std::cerr << "YAML log does not contain 'trajectory_log'!" << std::endl;
                    control_flag = 0;
                    return 0;
                }
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                for (int j = 0; j < 6; ++j) q_send[1][j] = planned_joint_trajectory[trajectory_index][j];

                if (!isSimulation)
                {
                    Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);
                }
                else
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj) q_recv[1][motorj] = q_send[1][motorj];
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
                ROS_INFO("MPC trajectory execution completed.");
                control_flag = 0;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        // 插值
        else if (control_flag == 777)
        {
            std::vector<std::vector<double>> q_mpc_goal = {
                {1.597743, 0.2950242, 2.156446, 3.101645, -0.4948243, -0.01648967, 1.0}, {2.041711, -0.616538, 2.032447, -1.33452, 1.159544, -2.898303, 1.0}, {1.20091, -1.84009, -1.90918, -1.54403, 1.20184, -0.859485, 1.0}, {-1.597743, 0.295025, 2.156445, -0.04025241, 0.4948952, 0.01637039, 1.0}};
            if (start_interp)
            {
                q_temp.resize(BRANCHN_N, std::vector<double>(MOTOR_BRANCHN_N, 0.0));
                q_temp = q_recv;  // 保存当前位置作为插值起点
                interp_step = 0;
                start_interp = false;
            }

            const int total_steps = 600;  // 30秒，200Hz
            double ratio = static_cast<double>(interp_step) / total_steps;

            // 插值计算 q_send = q_temp + ratio * (q_mpc_init - q_temp)
            for (int i = 0; i < BRANCHN_N; ++i)
            {
                for (int j = 0; j < MOTOR_BRANCHN_N; ++j)
                {
                    double start_val = q_temp[i][j];
                    double target_val = q_mpc_goal[i][j];
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
                control_flag = 0;

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

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}