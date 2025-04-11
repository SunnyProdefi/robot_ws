#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <ros/package.h>

std::string package_path = ros::package::getPath("robot_common");
std::string yaml_path = package_path + "/config/transform.yaml";
std::string path_tf_using = package_path + "/config/tf_using.yaml";

std::string robot_common_path = ros::package::getPath("robot_control");
std::string path_tf_obj = robot_common_path + "/config/common_tf.yaml";

void saveTransformToYAML(const std::string& filename, const Eigen::Matrix4f& transform_matrix_L, const Eigen::Matrix4f& transform_matrix_R)
{
    YAML::Emitter out;
    out << YAML::BeginMap;

    // 写入 target_pose_R
    out << YAML::Key << "target_pose_R" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginSeq << transform_matrix_R(0, 3) << transform_matrix_R(1, 3) << transform_matrix_R(2, 3) << YAML::EndSeq;
    out << YAML::Key << "orientation" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i)
    {
        out << YAML::Flow << YAML::BeginSeq;
        for (int j = 0; j < 3; ++j)
        {
            out << transform_matrix_R(i, j);
        }
        out << YAML::EndSeq;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    // 写入 target_pose_L
    out << YAML::Key << "target_pose_L" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginSeq << transform_matrix_L(0, 3) << transform_matrix_L(1, 3) << transform_matrix_L(2, 3) << YAML::EndSeq;
    out << YAML::Key << "orientation" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i)
    {
        out << YAML::Flow << YAML::BeginSeq;
        for (int j = 0; j < 3; ++j)
        {
            out << transform_matrix_L(i, j);
        }
        out << YAML::EndSeq;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    out << YAML::EndMap;

    // 直接覆盖文件，避免重复数据
    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    if (file.is_open())
    {
        file << out.c_str();
        file.close();
        ROS_INFO_STREAM("Saved latest transformation to " << filename);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to open file " << filename);
    }
}

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
        tf::StampedTransform transform, transform2;
        tf::StampedTransform tf_world_flan1, tf_world_flan4;
        tf::StampedTransform tf_base_link1_0, tf_base_link4_0;
        tf::StampedTransform tf_world_obj, tf_world_obj_1;
        try
        {
            listener.lookupTransform("world", "dummy_point3", ros::Time(0), transform);
            listener.lookupTransform("world", "dummy_point4", ros::Time(0), transform2);
            listener.lookupTransform("world", "flan1", ros::Time(0), tf_world_flan1);
            listener.lookupTransform("world", "flan4", ros::Time(0), tf_world_flan4);
            listener.lookupTransform("base_link", "Link1_0", ros::Time(0), tf_base_link1_0);
            listener.lookupTransform("base_link", "Link4_0", ros::Time(0), tf_base_link4_0);
            listener.lookupTransform("base_link", "Link2_0", ros::Time(0), tf_base_link1_0);
            listener.lookupTransform("base_link", "Link3_0", ros::Time(0), tf_base_link4_0);
            listener.lookupTransform("world", "object", ros::Time(0), tf_world_obj);
            listener.lookupTransform("world", "object1", ros::Time(0), tf_world_obj_1);

            Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f transform_matrix2 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_world_flan1 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_world_flan4 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_base_link1_0 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_base_link4_0 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_base_link2_0 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_base_link3_0 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_world_obj = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_world_obj_1 = Eigen::Matrix4f::Identity();

            tf::Matrix3x3 rotation_matrix(transform.getRotation());
            tf::Matrix3x3 rotation_matrix2(transform2.getRotation());
            tf::Matrix3x3 rot_mat_world_flan1(tf_world_flan1.getRotation());
            tf::Matrix3x3 rot_mat_world_flan4(tf_world_flan4.getRotation());
            tf::Matrix3x3 rot_mat_base_link1_0(tf_base_link1_0.getRotation());
            tf::Matrix3x3 rot_mat_base_link4_0(tf_base_link4_0.getRotation());
            tf::Matrix3x3 rot_mat_base_link2_0(tf_base_link1_0.getRotation());
            tf::Matrix3x3 rot_mat_base_link3_0(tf_base_link4_0.getRotation());
            tf::Matrix3x3 rot_mat_world_obj(tf_world_obj.getRotation());
            tf::Matrix3x3 rot_mat_world_obj_1(tf_world_obj_1.getRotation());

            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    transform_matrix(i, j) = rotation_matrix[i][j];
                    transform_matrix2(i, j) = rotation_matrix2[i][j];
                    tf_mat_world_flan1(i, j) = rot_mat_world_flan1[i][j];
                    tf_mat_world_flan4(i, j) = rot_mat_world_flan4[i][j];
                    tf_mat_base_link1_0(i, j) = rot_mat_base_link1_0[i][j];
                    tf_mat_base_link4_0(i, j) = rot_mat_base_link4_0[i][j];
                    tf_mat_base_link2_0(i, j) = rot_mat_base_link2_0[i][j];
                    tf_mat_base_link3_0(i, j) = rot_mat_base_link3_0[i][j];
                    tf_mat_world_obj(i, j) = rot_mat_world_obj[i][j];
                    tf_mat_world_obj_1(i, j) = rot_mat_world_obj_1[i][j];
                }
            }

            transform_matrix(0, 3) = transform.getOrigin().x();
            transform_matrix(1, 3) = transform.getOrigin().y();
            transform_matrix(2, 3) = transform.getOrigin().z();

            transform_matrix2(0, 3) = transform2.getOrigin().x();
            transform_matrix2(1, 3) = transform2.getOrigin().y();
            transform_matrix2(2, 3) = transform2.getOrigin().z();

            tf_mat_world_flan1(0, 3) = tf_world_flan1.getOrigin().x();
            tf_mat_world_flan1(1, 3) = tf_world_flan1.getOrigin().y();
            tf_mat_world_flan1(2, 3) = tf_world_flan1.getOrigin().z();

            tf_mat_world_flan4(0, 3) = tf_world_flan4.getOrigin().x();
            tf_mat_world_flan4(1, 3) = tf_world_flan4.getOrigin().y();
            tf_mat_world_flan4(2, 3) = tf_world_flan4.getOrigin().z();

            tf_mat_base_link1_0(0, 3) = tf_base_link1_0.getOrigin().x();
            tf_mat_base_link1_0(1, 3) = tf_base_link1_0.getOrigin().y();
            tf_mat_base_link1_0(2, 3) = tf_base_link1_0.getOrigin().z();

            tf_mat_base_link4_0(0, 3) = tf_base_link4_0.getOrigin().x();
            tf_mat_base_link4_0(1, 3) = tf_base_link4_0.getOrigin().y();
            tf_mat_base_link4_0(2, 3) = tf_base_link4_0.getOrigin().z();

            tf_mat_base_link2_0(0, 3) = tf_base_link1_0.getOrigin().x();
            tf_mat_base_link2_0(1, 3) = tf_base_link1_0.getOrigin().y();
            tf_mat_base_link2_0(2, 3) = tf_base_link1_0.getOrigin().z();

            tf_mat_base_link3_0(0, 3) = tf_base_link4_0.getOrigin().x();
            tf_mat_base_link3_0(1, 3) = tf_base_link4_0.getOrigin().y();
            tf_mat_base_link3_0(2, 3) = tf_base_link4_0.getOrigin().z();

            tf_mat_world_obj(0, 3) = tf_world_obj.getOrigin().x();
            tf_mat_world_obj(1, 3) = tf_world_obj.getOrigin().y();
            tf_mat_world_obj(2, 3) = tf_world_obj.getOrigin().z();

            tf_mat_world_obj_1(0, 3) = tf_world_obj_1.getOrigin().x();
            tf_mat_world_obj_1(1, 3) = tf_world_obj_1.getOrigin().y();
            tf_mat_world_obj_1(2, 3) = tf_world_obj_1.getOrigin().z();

            saveTransformToYAML(yaml_path, transform_matrix, transform_matrix2);
            saveTFToYAML(path_tf_using, tf_mat_world_flan1, "tf_mat_world_flan1");
            saveTFToYAML(path_tf_using, tf_mat_world_flan4, "tf_mat_world_flan4");
            saveTFToYAML(path_tf_using, tf_mat_base_link1_0, "tf_mat_base_link1_0");
            saveTFToYAML(path_tf_using, tf_mat_base_link4_0, "tf_mat_base_link4_0");
            saveTFToYAML(path_tf_using, tf_mat_base_link2_0, "tf_mat_base_link2_0");
            saveTFToYAML(path_tf_using, tf_mat_base_link3_0, "tf_mat_base_link3_0");

            saveTFToYAML(path_tf_obj, tf_mat_base_link1_0, "tf_mat_base_link1_0");
            saveTFToYAML(path_tf_obj, tf_mat_base_link2_0, "tf_mat_base_link2_0");
            saveTFToYAML(path_tf_obj, tf_mat_base_link3_0, "tf_mat_base_link3_0");
            saveTFToYAML(path_tf_obj, tf_mat_base_link4_0, "tf_mat_base_link4_0");
            saveTFToYAML(path_tf_obj, tf_mat_world_obj, "tf_mat_world_obj");
            saveTFToYAML(path_tf_obj, tf_mat_world_obj_1, "tf_mat_world_obj_1");
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        rate.sleep();
    }
    return 0;
}