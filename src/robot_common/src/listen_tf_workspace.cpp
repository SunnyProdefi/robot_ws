#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>

std::string robot_planning_path = ros::package::getPath("robot_planning");
std::string path_tf_workspace = robot_planning_path + "/config/workspace.yaml";

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <ros/ros.h>

void saveTFToYAML(const std::string& filename, const Eigen::Matrix4f& transform_matrix, const std::string& tf_name)
{
    YAML::Node root;

    // **尝试读取已有的 YAML 文件**
    std::ifstream file_in(filename);
    if (file_in.is_open())
    {
        try
        {
            root = YAML::Load(file_in);
        }
        catch (const YAML::Exception& e)
        {
            ROS_WARN_STREAM("Failed to load existing YAML, creating new file. Error: " << e.what());
            root = YAML::Node();
        }
        file_in.close();
    }

    // **创建 position 数据**
    YAML::Node position;
    position.push_back(transform_matrix(0, 3));
    position.push_back(transform_matrix(1, 3));
    position.push_back(transform_matrix(2, 3));
    position.SetStyle(YAML::EmitterStyle::Flow);  // **修正类型**

    // **创建 orientation 数据**
    YAML::Node orientation;
    for (int i = 0; i < 3; ++i)
    {
        YAML::Node row;
        for (int j = 0; j < 3; ++j)
        {
            row.push_back(transform_matrix(i, j));
        }
        row.SetStyle(YAML::EmitterStyle::Flow);  // **修正类型**
        orientation.push_back(row);
    }

    // **添加到 YAML 结构**
    root[tf_name]["target_pose"]["position"] = position;
    root[tf_name]["target_pose"]["orientation"] = orientation;

    // **写回 YAML 文件**
    std::ofstream file_out(filename, std::ios::out | std::ios::trunc);
    if (file_out.is_open())
    {
        file_out << root;
        file_out.close();
        ROS_INFO_STREAM("Saved transformation " << tf_name << " to " << filename);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to open file " << filename);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_transform_node");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Rate rate(10.0);
    // 显示等待信息
    ROS_INFO("Waiting for TF transform between 'world' and 'dummy_point3'...");

    // 显式等待 tf 可用（等待某个 tf 出现）
    bool tf_ready = listener.waitForTransform("world", "dummy_point3", ros::Time(0), ros::Duration(10.0));
    if (!tf_ready)
    {
        ROS_ERROR("Timeout: TF between 'world' and 'dummy_point3' not available.");
        return 1;
    }
    else
    {
        ROS_INFO("TF between 'world' and 'dummy_point3' is now available.");
    }
    while (ros::ok())
    {
        tf::StampedTransform tf_base_link1_0, tf_base_link4_0;
        tf::StampedTransform tf_base_link2_0, tf_base_link3_0;
        tf::StampedTransform tf_flan1_flan4;
        try
        {
            listener.lookupTransform("base_link", "Link1_0", ros::Time(0), tf_base_link1_0);
            listener.lookupTransform("base_link", "Link4_0", ros::Time(0), tf_base_link4_0);
            listener.lookupTransform("base_link", "Link2_0", ros::Time(0), tf_base_link2_0);
            listener.lookupTransform("base_link", "Link3_0", ros::Time(0), tf_base_link3_0);
            listener.lookupTransform("flan1", "flan4", ros::Time(0), tf_flan1_flan4);

            Eigen::Matrix4f tf_mat_base_link1_0 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_base_link4_0 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_flan1_flan4 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_base_link2_0 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_base_link3_0 = Eigen::Matrix4f::Identity();

            tf::Matrix3x3 rot_mat_base_link1_0(tf_base_link1_0.getRotation());
            tf::Matrix3x3 rot_mat_base_link4_0(tf_base_link4_0.getRotation());
            tf::Matrix3x3 rot_mat_flan1_flan4(tf_flan1_flan4.getRotation());
            tf::Matrix3x3 rot_mat_base_link2_0(tf_base_link2_0.getRotation());
            tf::Matrix3x3 rot_mat_base_link3_0(tf_base_link3_0.getRotation());

            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    tf_mat_base_link1_0(i, j) = rot_mat_base_link1_0[i][j];
                    tf_mat_base_link4_0(i, j) = rot_mat_base_link4_0[i][j];
                    tf_mat_flan1_flan4(i, j) = rot_mat_flan1_flan4[i][j];
                    tf_mat_base_link2_0(i, j) = rot_mat_base_link2_0[i][j];
                    tf_mat_base_link3_0(i, j) = rot_mat_base_link3_0[i][j];
                }
            }

            tf_mat_base_link1_0(0, 3) = tf_base_link1_0.getOrigin().x();
            tf_mat_base_link1_0(1, 3) = tf_base_link1_0.getOrigin().y();
            tf_mat_base_link1_0(2, 3) = tf_base_link1_0.getOrigin().z();

            tf_mat_base_link4_0(0, 3) = tf_base_link4_0.getOrigin().x();
            tf_mat_base_link4_0(1, 3) = tf_base_link4_0.getOrigin().y();
            tf_mat_base_link4_0(2, 3) = tf_base_link4_0.getOrigin().z();

            tf_mat_flan1_flan4(0, 3) = tf_flan1_flan4.getOrigin().x();
            tf_mat_flan1_flan4(1, 3) = tf_flan1_flan4.getOrigin().y();
            tf_mat_flan1_flan4(2, 3) = tf_flan1_flan4.getOrigin().z();

            tf_mat_base_link2_0(0, 3) = tf_base_link2_0.getOrigin().x();
            tf_mat_base_link2_0(1, 3) = tf_base_link2_0.getOrigin().y();
            tf_mat_base_link2_0(2, 3) = tf_base_link2_0.getOrigin().z();

            tf_mat_base_link3_0(0, 3) = tf_base_link3_0.getOrigin().x();
            tf_mat_base_link3_0(1, 3) = tf_base_link3_0.getOrigin().y();
            tf_mat_base_link3_0(2, 3) = tf_base_link3_0.getOrigin().z();

            saveTFToYAML(path_tf_workspace, tf_mat_base_link1_0, "tf_mat_base_link1_0");
            saveTFToYAML(path_tf_workspace, tf_mat_base_link4_0, "tf_mat_base_link4_0");
            saveTFToYAML(path_tf_workspace, tf_mat_flan1_flan4, "tf_mat_flan1_flan4");
            saveTFToYAML(path_tf_workspace, tf_mat_base_link2_0, "tf_mat_base_link2_0");
            saveTFToYAML(path_tf_workspace, tf_mat_base_link3_0, "tf_mat_base_link3_0");
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        rate.sleep();
    }
    return 0;
}