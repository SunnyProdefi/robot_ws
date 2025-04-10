#include <ros/ros.h>
#include <robot_planning/RobotPose.h>
#include <robot_planning/ikfast_wrapper_single_arm.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace Eigen;

class RobotPoseServer
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    robots::Kinematics kinematics_branch2_;
    robots::Kinematics kinematics_branch3_;

    // 从YAML文件加载变换矩阵
    Matrix4d loadTransformFromYAML(const std::string& file_path, const std::string& transform_name)
    {
        Matrix4d transform = Matrix4d::Identity();
        try
        {
            YAML::Node config = YAML::LoadFile(file_path);
            if (config[transform_name])
            {
                const auto& matrix = config[transform_name];
                for (int i = 0; i < 4; ++i)
                {
                    for (int j = 0; j < 4; ++j)
                    {
                        transform(i, j) = matrix[i][j].as<double>();
                    }
                }
            }
        }
        catch (const YAML::Exception& e)
        {
            ROS_ERROR("Failed to load transform from YAML: %s", e.what());
        }
        return transform;
    }

    // 将位姿转换为变换矩阵
    Matrix4d poseToTransform(const std::vector<double>& pose)
    {
        Matrix4d transform = Matrix4d::Identity();

        // 提取位置
        transform(0, 3) = pose[0];
        transform(1, 3) = pose[1];
        transform(2, 3) = pose[2];

        // 提取四元数并转换为旋转矩阵
        double qw = pose[6], qx = pose[3], qy = pose[4], qz = pose[5];
        double n = 1.0 / std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        qw *= n;
        qx *= n;
        qy *= n;
        qz *= n;

        transform(0, 0) = 1 - 2 * qy * qy - 2 * qz * qz;
        transform(0, 1) = 2 * qx * qy - 2 * qz * qw;
        transform(0, 2) = 2 * qx * qz + 2 * qy * qw;

        transform(1, 0) = 2 * qx * qy + 2 * qz * qw;
        transform(1, 1) = 1 - 2 * qx * qx - 2 * qz * qz;
        transform(1, 2) = 2 * qy * qz - 2 * qx * qw;

        transform(2, 0) = 2 * qx * qz - 2 * qy * qw;
        transform(2, 1) = 2 * qy * qz + 2 * qx * qw;
        transform(2, 2) = 1 - 2 * qx * qx - 2 * qy * qy;

        return transform;
    }

    // 将变换矩阵转换为位姿
    std::vector<double> transformToPose(const Matrix4d& transform)
    {
        std::vector<double> pose(7);

        // 提取位置
        pose[0] = transform(0, 3);
        pose[1] = transform(1, 3);
        pose[2] = transform(2, 3);

        // 提取旋转矩阵并转换为四元数
        Matrix3d rot = transform.block<3, 3>(0, 0);
        double trace = rot.trace();
        if (trace > 0)
        {
            double s = 0.5 / std::sqrt(trace + 1.0);
            pose[3] = 0.25 / s;
            pose[4] = (rot(2, 1) - rot(1, 2)) * s;
            pose[5] = (rot(0, 2) - rot(2, 0)) * s;
            pose[6] = (rot(1, 0) - rot(0, 1)) * s;
        }
        else
        {
            if (rot(0, 0) > rot(1, 1) && rot(0, 0) > rot(2, 2))
            {
                double s = 2.0 * std::sqrt(1.0 + rot(0, 0) - rot(1, 1) - rot(2, 2));
                pose[3] = (rot(2, 1) - rot(1, 2)) / s;
                pose[4] = 0.25 * s;
                pose[5] = (rot(0, 1) + rot(1, 0)) / s;
                pose[6] = (rot(0, 2) + rot(2, 0)) / s;
            }
            else if (rot(1, 1) > rot(2, 2))
            {
                double s = 2.0 * std::sqrt(1.0 + rot(1, 1) - rot(0, 0) - rot(2, 2));
                pose[3] = (rot(0, 2) - rot(2, 0)) / s;
                pose[4] = (rot(0, 1) + rot(1, 0)) / s;
                pose[5] = 0.25 * s;
                pose[6] = (rot(1, 2) + rot(2, 1)) / s;
            }
            else
            {
                double s = 2.0 * std::sqrt(1.0 + rot(2, 2) - rot(0, 0) - rot(1, 1));
                pose[3] = (rot(1, 0) - rot(0, 1)) / s;
                pose[4] = (rot(0, 2) + rot(2, 0)) / s;
                pose[5] = (rot(1, 2) + rot(2, 1)) / s;
                pose[6] = 0.25 * s;
            }
        }

        return pose;
    }

    bool handlePoseRequest(robot_planning::RobotPose::Request& req, robot_planning::RobotPose::Response& res)
    {
        try
        {
            // ---------- 读取变换矩阵 ----------
            const Matrix4d tf_base_link2_0 = loadTransformFromYAML("tf_using.yaml", "tf_mat_base_link2_0");
            const Matrix4d tf_base_link3_0 = loadTransformFromYAML("tf_using.yaml", "tf_mat_base_link3_0");

            // ---------- 获取 float_base 的世界变换 ----------
            const Matrix4d tf_world_float_base = poseToTransform(req.float_base_pose);

            // ---------- 正向运动学：计算每个分支末端在各自 link0 下的位置 ----------
            const std::vector<float> branch2_joints_float(req.branch2_joints.begin(), req.branch2_joints.end());
            const std::vector<float> branch3_joints_float(req.branch3_joints.begin(), req.branch3_joints.end());

            const std::vector<float> branch2_pose = kinematics_branch2_.forward(branch2_joints_float);
            const std::vector<float> branch3_pose = kinematics_branch3_.forward(branch3_joints_float);

            const Matrix4d tf_link0_branch2 = poseToTransform(std::vector<double>(branch2_pose.begin(), branch2_pose.end()));
            const Matrix4d tf_link0_branch3 = poseToTransform(std::vector<double>(branch3_pose.begin(), branch3_pose.end()));

            // ---------- 构建完整的世界 → 分支末端 变换链 ----------
            const Matrix4d tf_world_branch2 = tf_world_float_base * tf_base_link2_0 * tf_link0_branch2;
            const Matrix4d tf_world_branch3 = tf_world_float_base * tf_base_link3_0 * tf_link0_branch3;

            // ---------- link2_0 → branch2_end 变换（相对 base）----------
            const Matrix4d tf_link2_0_branch2 = tf_base_link2_0.inverse() * tf_link0_branch2;
            const Matrix4d tf_link3_0_branch3 = tf_base_link3_0.inverse() * tf_link0_branch3;

            // ---------- 构建帧变换表 ----------
            std::map<std::pair<std::string, std::string>, Matrix4d> frame_transforms = {{{"world", "branch2_end"}, tf_world_branch2},
                                                                                        {{"world", "branch3_end"}, tf_world_branch3},
                                                                                        {{"branch2_end", "world"}, tf_world_branch2.inverse()},
                                                                                        {{"branch3_end", "world"}, tf_world_branch3.inverse()},

                                                                                        {{"branch2_end", "branch3_end"}, tf_world_branch2.inverse() * tf_world_branch3},
                                                                                        {{"branch3_end", "branch2_end"}, tf_world_branch3.inverse() * tf_world_branch2},

                                                                                        {{"link2_0", "branch2_end"}, tf_link2_0_branch2},
                                                                                        {{"branch2_end", "link2_0"}, tf_link2_0_branch2.inverse()},

                                                                                        {{"link3_0", "branch3_end"}, tf_link3_0_branch3},
                                                                                        {{"branch3_end", "link3_0"}, tf_link3_0_branch3.inverse()}};

            const auto key = std::make_pair(req.source_frame, req.target_frame);
            if (frame_transforms.find(key) != frame_transforms.end())
            {
                res.transform = transformToPose(frame_transforms[key]);
                res.success = true;
                res.message = "Successfully computed transform";
            }
            else
            {
                res.success = false;
                res.message = "Unsupported frame pair: " + req.source_frame + " -> " + req.target_frame;
            }
        }
        catch (const std::exception& e)
        {
            res.success = false;
            res.message = std::string("Exception occurred: ") + e.what();
        }

        return true;
    }

public:
    RobotPoseServer() { service_ = nh_.advertiseService("robot_pose", &RobotPoseServer::handlePoseRequest, this); }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_pose_server");
    RobotPoseServer server;
    ROS_INFO("Robot pose server is ready");
    ros::spin();
    return 0;
}