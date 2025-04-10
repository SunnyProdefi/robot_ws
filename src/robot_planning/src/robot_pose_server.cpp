// robot_pose_server.cpp (使用 tf2 + Eigen 重构 + ROS_INFO 调试)
#include <ros/ros.h>
#include <ros/package.h>
#include <robot_planning/RobotPose.h>
#include <robot_planning/ikfast_wrapper_single_arm.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <map>

using namespace Eigen;

namespace utils
{
    Eigen::Isometry3d poseToTransform(const std::vector<double> &pose)
    {
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        if (pose.size() < 7)
            return tf;

        Eigen::Vector3d pos(pose[0], pose[1], pose[2]);
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);  // (w, x, y, z)
        q.normalize();
        tf.linear() = q.toRotationMatrix();
        tf.translation() = pos;
        return tf;
    }

    std::vector<double> transformToPose(const Eigen::Isometry3d &tf)
    {
        std::vector<double> pose(7);
        Eigen::Vector3d pos = tf.translation();
        Eigen::Quaterniond q(tf.rotation());
        pose[0] = pos.x();
        pose[1] = pos.y();
        pose[2] = pos.z();
        pose[3] = q.x();
        pose[4] = q.y();
        pose[5] = q.z();
        pose[6] = q.w();
        return pose;
    }

    Eigen::Isometry3d loadTransformFromYAML(const std::string &file_path, const std::string &name)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        try
        {
            YAML::Node config = YAML::LoadFile(file_path);
            YAML::Node pose = config[name]["target_pose"];
            if (!pose)
                return transform;

            Eigen::Vector3d position;
            for (int i = 0; i < 3; ++i) position[i] = pose["position"][i].as<double>();

            Eigen::Matrix3d rot;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) rot(i, j) = pose["orientation"][i][j].as<double>();

            transform.linear() = rot;
            transform.translation() = position;
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("YAML load error: %s", e.what());
        }
        return transform;
    }
}  // namespace utils

class RobotPoseServer
{
    ros::NodeHandle nh_;
    ros::ServiceServer srv_;
    robots::Kinematics kin2_, kin3_;

public:
    RobotPoseServer() { srv_ = nh_.advertiseService("robot_pose", &RobotPoseServer::callback, this); }

    bool callback(robot_planning::RobotPose::Request &req, robot_planning::RobotPose::Response &res)
    {
        ROS_INFO("[RobotPoseServer] Callback triggered for source: %s, target: %s", req.source_frame.c_str(), req.target_frame.c_str());

        using namespace utils;

        if (req.float_base_pose.size() < 7 || req.branch2_joints.size() < 6 || req.branch3_joints.size() < 6)
        {
            ROS_WARN("Invalid input sizes: float_base_pose=%lu, branch2_joints=%lu, branch3_joints=%lu", req.float_base_pose.size(), req.branch2_joints.size(), req.branch3_joints.size());
            res.success = false;
            res.message = "Invalid pose or joint vector size.";
            return true;
        }
        std::string tf_using_path = ros::package::getPath("robot_planning") + "/config/tf_using.yaml";
        Eigen::Isometry3d T_base2 = loadTransformFromYAML(tf_using_path, "tf_mat_base_link2_0");
        Eigen::Isometry3d T_base3 = loadTransformFromYAML(tf_using_path, "tf_mat_base_link3_0");
        Eigen::Isometry3d T_world_base = poseToTransform(req.float_base_pose);
        // ROSINFO打印
        Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]");

        ROS_INFO_STREAM("T_base2:\n" << T_base2.matrix().format(fmt));
        ROS_INFO_STREAM("T_base3:\n" << T_base3.matrix().format(fmt));
        ROS_INFO_STREAM("T_world_base:\n" << T_world_base.matrix().format(fmt));

        auto j2 = std::vector<float>(req.branch2_joints.begin(), req.branch2_joints.begin() + 6);
        auto j3 = std::vector<float>(req.branch3_joints.begin(), req.branch3_joints.begin() + 6);

        // ROSINFO打印j2和j3
        std::ostringstream oss_j2, oss_j3;
        for (const auto &val : j2) oss_j2 << val << " ";
        for (const auto &val : j3) oss_j3 << val << " ";
        ROS_INFO("j2: %s", oss_j2.str().c_str());
        ROS_INFO("j3: %s", oss_j3.str().c_str());

        ROS_INFO("Computing forward kinematics for branch 2 and 3...");

        // 分别获取关节角度的末端位姿
        std::vector<float> ee_pose_2 = kin2_.forward(j2);  // 应该返回长度为12的向量
        std::vector<float> ee_pose_3 = kin3_.forward(j3);

        if (ee_pose_2.size() != 12 || ee_pose_3.size() != 12)
        {
            ROS_ERROR("Invalid FK result for branch2 or branch3");
            return false;
        }

        // 构造 T_link0_flan2 为 Isometry3d
        Eigen::Isometry3d T_l0_b2 = Eigen::Isometry3d::Identity();
        T_l0_b2.linear() << ee_pose_2[0], ee_pose_2[1], ee_pose_2[2], ee_pose_2[4], ee_pose_2[5], ee_pose_2[6], ee_pose_2[8], ee_pose_2[9], ee_pose_2[10];
        T_l0_b2.translation() << ee_pose_2[3], ee_pose_2[7], ee_pose_2[11];

        // 构造 T_link0_flan3 为 Isometry3d
        Eigen::Isometry3d T_l0_b3 = Eigen::Isometry3d::Identity();
        T_l0_b3.linear() << ee_pose_3[0], ee_pose_3[1], ee_pose_3[2], ee_pose_3[4], ee_pose_3[5], ee_pose_3[6], ee_pose_3[8], ee_pose_3[9], ee_pose_3[10];
        T_l0_b3.translation() << ee_pose_3[3], ee_pose_3[7], ee_pose_3[11];

        // ROSINFO 打印 T_l0_b2 和 T_l0_b3
        ROS_INFO_STREAM("T_l0_b2:\n" << T_l0_b2.matrix().format(fmt));
        ROS_INFO_STREAM("T_l0_b3:\n" << T_l0_b3.matrix().format(fmt));

        // 计算 world 下的位置
        Eigen::Isometry3d T_world_b2 = T_world_base * T_base2 * T_l0_b2;
        Eigen::Isometry3d T_world_b3 = T_world_base * T_base3 * T_l0_b3;

        // ROSINFO 打印 T_world_b2 和 T_world_b3
        ROS_INFO_STREAM("T_world_b2:\n" << T_world_b2.matrix().format(fmt));
        ROS_INFO_STREAM("T_world_b3:\n" << T_world_b3.matrix().format(fmt));

        // 构建 tf_map
        std::map<std::pair<std::string, std::string>, Eigen::Isometry3d> tf_map = {{{"world", "branch2_end"}, T_world_b2},
                                                                                   {{"world", "branch3_end"}, T_world_b3},
                                                                                   {{"branch2_end", "world"}, T_world_b2.inverse()},
                                                                                   {{"branch3_end", "world"}, T_world_b3.inverse()},
                                                                                   {{"branch2_end", "branch3_end"}, T_world_b2.inverse() * T_world_b3},
                                                                                   {{"branch3_end", "branch2_end"}, T_world_b3.inverse() * T_world_b2},
                                                                                   {{"link2_0", "branch2_end"}, T_base2.inverse() * T_l0_b2},
                                                                                   {{"branch2_end", "link2_0"}, T_l0_b2.inverse() * T_base2},
                                                                                   {{"link3_0", "branch3_end"}, T_base3.inverse() * T_l0_b3},
                                                                                   {{"branch3_end", "link3_0"}, T_l0_b3.inverse() * T_base3}};

        auto key = std::make_pair(req.source_frame, req.target_frame);
        if (tf_map.count(key))
        {
            ROS_INFO("Transform from %s to %s successfully computed.", req.source_frame.c_str(), req.target_frame.c_str());
            res.transform = transformToPose(tf_map[key]);
            res.success = true;
            res.message = "Transform computed.";
        }
        else
        {
            ROS_WARN("Requested unsupported transform: %s -> %s", req.source_frame.c_str(), req.target_frame.c_str());
            res.success = false;
            res.message = "Unsupported frame pair.";
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_pose_server");
    RobotPoseServer server;
    ROS_INFO("Robot pose server ready.");
    ros::spin();
    return 0;
}