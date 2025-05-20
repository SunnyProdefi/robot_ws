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

std_msgs::Float64MultiArray gripper_command;

std_msgs::Float64MultiArray motor_state;

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

// float_base位置
std::vector<double> float_base_position = {0, 0, 0.45, 0, 0.7071, 0, 0.7071};

// grasp_object位置
std::vector<double> grasp_object_position = {0.5825, 0.36, -0.07, 0, 0, -0.258734, 0.965946};

// grasp_cube_object位置
std::vector<double> grasp_cube_object_position = {0.6925, -0.1325, 0.15, 0, 0, 0, 1};

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

    // grasp_cube_object位置发布者
    ros::Publisher grasp_cube_object_pub = nh.advertise<geometry_msgs::Pose>("/grasp_object_cube_state", 10);

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
        motor_state.data.resize(BRANCHN_N * MOTOR_BRANCHN_N);
        for (int b = 0; b < BRANCHN_N; ++b)
            for (int j = 0; j < MOTOR_BRANCHN_N; ++j) motor_state.data[b * MOTOR_BRANCHN_N + j] = q_recv[b][j];
        motor_state_pub.publish(motor_state);
    };

    auto computeAndPublishGraspPose = [&](const Eigen::Matrix4d &goal_tf_mat_base_link2_0)
    {
        // 获取 EE Pose
        Eigen::Matrix4d tf_mat_link2_0_flan2, tf_mat_link3_0_flan3;
        if (!getCurrentEEPose(tf_mat_link2_0_flan2, tf_mat_link3_0_flan3))
        {
            control_flag = 0;
            return;
        }

        // std::cout << "tf_mat_link2_0_flan2:\n" << tf_mat_link2_0_flan2 << std::endl;
        // std::cout << "tf_mat_link3_0_flan3:\n" << tf_mat_link3_0_flan3 << std::endl;

        // 构造世界→base
        Eigen::Vector3d trans_base(float_base_position[0], float_base_position[1], float_base_position[2]);
        Eigen::Quaterniond quat_base(float_base_position[6], float_base_position[3], float_base_position[4], float_base_position[5]);
        Eigen::Matrix3d rot_base = quat_base.normalized().toRotationMatrix();
        Eigen::Matrix4d tf_mat_world_base = Eigen::Matrix4d::Identity();
        tf_mat_world_base.block<3, 3>(0, 0) = rot_base;
        tf_mat_world_base.block<3, 1>(0, 3) = trans_base;
        // std::cout << "tf_mat_world_base:\n" << tf_mat_world_base << std::endl;

        // flan2 → grasp_object_cube
        Eigen::Vector3d trans(0.118, -0.014, 0.352);
        Eigen::Quaterniond quat_cube(0.000, 0.000, 0.707, -0.707);
        Eigen::Matrix4d tf_mat_flan2_grasp_cube_object = Eigen::Matrix4d::Identity();
        tf_mat_flan2_grasp_cube_object.block<3, 3>(0, 0) = quat_cube.normalized().toRotationMatrix();
        tf_mat_flan2_grasp_cube_object.block<3, 1>(0, 3) = trans;
        // std::cout << "tf_mat_flan2_grasp_cube_object:\n" << tf_mat_flan2_grasp_cube_object << std::endl;

        // 计算 world → grasp_cube_object
        Eigen::Matrix4d tf_mat_world_grasp_cube_object = tf_mat_world_base * goal_tf_mat_base_link2_0 * tf_mat_link2_0_flan2 * tf_mat_flan2_grasp_cube_object;
        // std::cout << "goal_tf_mat_base_link2_0:\n" << goal_tf_mat_base_link2_0 << std::endl;
        // std::cout << "tf_mat_world_grasp_cube_object:\n" << tf_mat_world_grasp_cube_object << std::endl;

        Eigen::Vector3d position = tf_mat_world_grasp_cube_object.block<3, 1>(0, 3);
        Eigen::Matrix3d rotation = tf_mat_world_grasp_cube_object.block<3, 3>(0, 0);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R_corrected = svd.matrixU() * svd.matrixV().transpose();
        Eigen::Quaterniond quat(R_corrected);

        grasp_cube_object_position[0] = position.x();
        grasp_cube_object_position[1] = position.y();
        grasp_cube_object_position[2] = position.z();
        grasp_cube_object_position[3] = quat.x();
        grasp_cube_object_position[4] = quat.y();
        grasp_cube_object_position[5] = quat.z();
        grasp_cube_object_position[6] = quat.w();

        // std::cout << "grasp_cube_object_position [x, y, z, qx, qy, qz, qw]: ";
        // for (const auto &val : grasp_cube_object_position) std::cout << val << " ";
        // std::cout << std::endl;

        geometry_msgs::Pose grasp_cube_object_pose;
        grasp_cube_object_pose.position.x = grasp_cube_object_position[0];
        grasp_cube_object_pose.position.y = grasp_cube_object_position[1];
        grasp_cube_object_pose.position.z = grasp_cube_object_position[2];
        grasp_cube_object_pose.orientation.x = grasp_cube_object_position[3];
        grasp_cube_object_pose.orientation.y = grasp_cube_object_position[4];
        grasp_cube_object_pose.orientation.z = grasp_cube_object_position[5];
        grasp_cube_object_pose.orientation.w = grasp_cube_object_position[6];
        grasp_cube_object_pub.publish(grasp_cube_object_pose);
    };

    // 规划服务调用（初始化 + 请求发送）
    auto requestPlanning = [&](double dx, double dy, double dz) -> bool
    {
        robot_planning::PlanPathHome srv;

        std::vector<double> init_floating_base = float_base_position;
        std::vector<double> init_joint_angles;
        init_joint_angles.insert(init_joint_angles.end(), q_recv[1].begin(), q_recv[1].begin() + 6);  // 左臂6个
        init_joint_angles.insert(init_joint_angles.end(), q_recv[2].begin(), q_recv[2].begin() + 6);  // 右臂6个

        std::vector<double> gold_floating_base = init_floating_base;
        if (gold_floating_base.size() >= 3)
        {
            gold_floating_base[0] += dx;
            gold_floating_base[1] += dy;
            gold_floating_base[2] += dz;
        }
        else
        {
            ROS_WARN_STREAM("init_floating_base size is too small.");
            return false;
        }

        srv.request.init_floating_base = init_floating_base;
        srv.request.init_joint_angles = init_joint_angles;
        srv.request.gold_floating_base = gold_floating_base;
        srv.request.gold_joint_angles = init_joint_angles;

        if (planning_client_home.call(srv) && srv.response.success)
        {
            ROS_INFO("Planning request sent and completed successfully");
            return true;
        }
        else
        {
            ROS_ERROR("Planning request failed.");
            return false;
        }
    };

    // 读取规划结果
    auto loadPlanningResult = [&]() -> bool
    {
        std::string path = ros::package::getPath("robot_planning") + "/config/planning_result_home.yaml";
        std::ifstream file(path);
        if (!file.good())
            return false;

        YAML::Node config = YAML::LoadFile(path);
        if (config["joint_angle_sequence"])
        {
            planned_joint_trajectory.clear();
            for (const auto &point : config["joint_angle_sequence"])
            {
                std::vector<double> angles;
                for (const auto &angle : point) angles.push_back(angle.as<double>());
                planned_joint_trajectory.push_back(angles);
            }
        }

        if (config["floating_base_sequence"])
        {
            floating_base_sequence.clear();
            for (const auto &pose : config["floating_base_sequence"])
            {
                std::vector<double> base_state;
                for (const auto &val : pose) base_state.push_back(val.as<double>());
                floating_base_sequence.push_back(base_state);
            }
        }

        ROS_INFO("Loaded planning result: %zu steps", planned_joint_trajectory.size());
        return true;
    };

    // 更新电机状态并发布
    auto publishMotorState = [&]()
    {
        motor_state.data.resize(BRANCHN_N * MOTOR_BRANCHN_N);
        for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
            for (int motorj = 0; motorj < MOTOR_BRANCHN_N; ++motorj) motor_state.data[branchi * MOTOR_BRANCHN_N + motorj] = q_recv[branchi][motorj];

        motor_state_pub.publish(motor_state);
    };

    // 执行一步轨迹并发布基座姿态（仿真模式）
    auto executeSimStep = [&](int index)
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
        float_base_pose.position.x = floating_base_sequence[index][0];
        float_base_pose.position.y = floating_base_sequence[index][1];
        float_base_pose.position.z = floating_base_sequence[index][2];
        float_base_pose.orientation.x = floating_base_sequence[index][3];
        float_base_pose.orientation.y = floating_base_sequence[index][4];
        float_base_pose.orientation.z = floating_base_sequence[index][5];
        float_base_pose.orientation.w = floating_base_sequence[index][6];
        float_base_pub.publish(float_base_pose);

        // 更新浮动基座位置
        float_base_position[0] = floating_base_sequence[index][0];
        float_base_position[1] = floating_base_sequence[index][1];
        float_base_position[2] = floating_base_sequence[index][2];
        float_base_position[3] = floating_base_sequence[index][3];
        float_base_position[4] = floating_base_sequence[index][4];
        float_base_position[5] = floating_base_sequence[index][5];
        float_base_position[6] = floating_base_sequence[index][6];
    };

    auto executeRealStep = [&](int index)
    {
        // 1. 设置关节命令
        Motor_SendRec_Func_ALL(MOTORCOMMAND_POSITION);

        // 2. 调用 base_link 位姿服务
        robot_control::GetBaseLinkPose srv;
        for (int j = 0; j < 6; ++j)
        {
            srv.request.joint_angles_branch1.push_back(planned_joint_trajectory[index][j]);
            srv.request.joint_angles_branch4.push_back(planned_joint_trajectory[index][18 + j]);
        }

        if (client.call(srv))
        {
            ROS_INFO("Base link pose calculated: Position (x: %f, y: %f, z: %f), Orientation (x: %f, y: %f, z: %f, w: %f)", srv.response.base_link_pose.position.x, srv.response.base_link_pose.position.y, srv.response.base_link_pose.position.z, srv.response.base_link_pose.orientation.x,
                     srv.response.base_link_pose.orientation.y, srv.response.base_link_pose.orientation.z, srv.response.base_link_pose.orientation.w);

            // 发布浮动基座位姿
            float_base_pub.publish(srv.response.base_link_pose);

            // 更新 float_base_position
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
            publishMotorState();

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

        // 双臂操作-运动到初始位姿
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
                gripper_command.data = {0.0, 1.0, 1.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;
                // 发布电机位置状态
                publishMotorState();
                // 延时
                ros::Duration(1.0).sleep();  // 延时1秒，确保夹爪运动完成
            }

            // 发布电机位置状态
            publishMotorState();
        }

        // 双臂操作-运动到抓取准备位姿
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
                for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                    {
                        q_send[branchi][motorj] = planned_joint_trajectory[trajectory_index][branchi * (MOTOR_BRANCHN_N - 1) + motorj];
                    }
                }

                // 执行当前帧
                if (isSimulation)
                {
                    executeSimStep(trajectory_index);
                }
                else
                {
                    executeRealStep(trajectory_index);
                }

                publishMotorState();
                ++trajectory_index;
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

        // 双臂操作-分支2、3移动到抓取位姿
        else if (control_flag == 3)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 3 received" << std::endl;
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
                TZp(2, 3) = 0.03;
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
                control_flag = 4;
                planning_requested = planning_completed = false;
                trajectory_index = 0;

                // 发布夹爪指令
                gripper_command.data = {0.0, 0.0, 0.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 0.4;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;
                // 发布电机位置状态
                publishMotorState();
                ros::Duration(1.0).sleep();  // 延时1秒，确保夹爪运动完成
            }
        }

        // 双臂操作-z 轴移动 0.12 米
        else if (control_flag == 4)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 4 received" << std::endl;
                planning_requested = requestPlanning(0.0, 0.0, 0.12);  // z轴移动 +0.12m
                if (!planning_requested)
                    control_flag = 0;
            }
            else if (!planning_completed)
            {
                planning_completed = loadPlanningResult();
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置关节角度指令
                for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                    {
                        q_send[branchi][motorj] = planned_joint_trajectory[trajectory_index][branchi * (MOTOR_BRANCHN_N - 1) + motorj];
                    }
                }

                // 执行当前帧
                if (isSimulation)
                {
                    executeSimStep(trajectory_index);
                    computeAndPublishGraspPose(goal_tf_mat_base_link2_0);
                }
                else
                {
                    executeRealStep(trajectory_index);
                }

                publishMotorState();
                ++trajectory_index;
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

        // 双臂操作-x 轴移动 -0.03 米
        else if (control_flag == 5)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 5 received" << std::endl;
                planning_requested = requestPlanning(-0.03, 0.0, 0.0);
                if (!planning_requested)
                    control_flag = 0;
            }
            else if (!planning_completed)
            {
                planning_completed = loadPlanningResult();
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 更新发送关节
                for (int i = 0; i < BRANCHN_N; ++i)
                    for (int j = 0; j < MOTOR_BRANCHN_N - 1; ++j) q_send[i][j] = planned_joint_trajectory[trajectory_index][i * (MOTOR_BRANCHN_N - 1) + j];

                if (isSimulation)
                {
                    executeSimStep(trajectory_index);
                    computeAndPublishGraspPose(goal_tf_mat_base_link2_0);
                }
                else
                {
                    executeRealStep(trajectory_index);
                }

                publishMotorState();
                ++trajectory_index;
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

        // 双臂操作-y 轴移动 0.1325 米
        else if (control_flag == 6)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 6 received" << std::endl;
                planning_requested = requestPlanning(0.0, 0.1325, 0.0);  // y轴 +0.1325m
                if (!planning_requested)
                    control_flag = 0;
            }
            else if (!planning_completed)
            {
                planning_completed = loadPlanningResult();
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                    {
                        q_send[branchi][motorj] = planned_joint_trajectory[trajectory_index][branchi * (MOTOR_BRANCHN_N - 1) + motorj];
                    }
                }

                if (isSimulation)
                {
                    executeSimStep(trajectory_index);
                    computeAndPublishGraspPose(goal_tf_mat_base_link2_0);
                }
                else
                {
                    executeRealStep(trajectory_index);
                }

                publishMotorState();
                ++trajectory_index;
            }
            else
            {
                ROS_INFO("Trajectory execution completed");
                control_flag = 7;
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;
            }
        }

        // 双臂操作-z 轴移动 -0.02 米
        else if (control_flag == 7)
        {
            if (!planning_requested)
            {
                std::cout << "Control flag 7 received" << std::endl;
                planning_requested = requestPlanning(0.0, 0.0, -0.02);  // z轴 -0.02m
                if (!planning_requested)
                    control_flag = 0;
            }
            else if (!planning_completed)
            {
                planning_completed = loadPlanningResult();
                trajectory_index = 0;
            }
            else if (trajectory_index < planned_joint_trajectory.size())
            {
                // 设置目标关节角度
                for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
                {
                    for (int motorj = 0; motorj < MOTOR_BRANCHN_N - 1; ++motorj)
                    {
                        q_send[branchi][motorj] = planned_joint_trajectory[trajectory_index][branchi * (MOTOR_BRANCHN_N - 1) + motorj];
                    }
                }

                // 执行当前帧
                if (isSimulation)
                {
                    executeSimStep(trajectory_index);
                    computeAndPublishGraspPose(goal_tf_mat_base_link2_0);
                }
                else
                {
                    executeRealStep(trajectory_index);
                }

                publishMotorState();
                ++trajectory_index;
            }
            else
            {
                ROS_INFO("Trajectory execution completed");
                planning_requested = false;
                planning_completed = false;
                trajectory_index = 0;

                // === 加入夹爪指令 ===
                gripper_command.data = {0.0, 1.0, 1.0, 0.0};
                gripper_pub.publish(gripper_command);

                q_recv[0][MOTOR_BRANCHN_N - 1] = 0.4;  // 更新夹爪状态
                q_recv[1][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[2][MOTOR_BRANCHN_N - 1] = 1.0;
                q_recv[3][MOTOR_BRANCHN_N - 1] = 0.4;

                publishMotorState();
                ros::Duration(1.0).sleep();  // 等待夹爪完成

                control_flag = 0;  // 流程结束
            }
        }

        else if (control_flag == 101)
        {
            // 发布夹爪指令
            gripper_command.data = {1.0, 1.0, 1.0, 1.0};
            gripper_pub.publish(gripper_command);

            q_recv[0][MOTOR_BRANCHN_N - 1] = 1.0;  // 更新夹爪状态
            q_recv[1][MOTOR_BRANCHN_N - 1] = 1.0;
            q_recv[2][MOTOR_BRANCHN_N - 1] = 1.0;
            q_recv[3][MOTOR_BRANCHN_N - 1] = 1.0;
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
        }

        else if (control_flag == 102)
        {
            // 发布夹爪指令
            gripper_command.data = {0.0, 0.5, 0.5, 0.0};
            gripper_pub.publish(gripper_command);

            q_recv[0][MOTOR_BRANCHN_N - 1] = 0.0;  // 更新夹爪状态
            q_recv[1][MOTOR_BRANCHN_N - 1] = 0.5;
            q_recv[2][MOTOR_BRANCHN_N - 1] = 0.5;
            q_recv[3][MOTOR_BRANCHN_N - 1] = 0.0;
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

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}