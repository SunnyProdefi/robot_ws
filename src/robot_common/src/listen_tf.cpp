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

std::string robot_planning_path = ros::package::getPath("robot_planning");
std::string path_tf_double_arm_float = robot_planning_path + "/config/ik_urdf_double_arm_float.yaml";

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <ros/ros.h>

void saveTransformToYAML(const std::string& filename, const Eigen::Matrix4f& transform_matrix_L, const Eigen::Matrix4f& transform_matrix_R)
{
    YAML::Node root;

    // ✅ 尝试读取已有文件
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

    // ✅ target_pose_R
    YAML::Node position_R;
    position_R.push_back(transform_matrix_R(0, 3));
    position_R.push_back(transform_matrix_R(1, 3));
    position_R.push_back(transform_matrix_R(2, 3));
    position_R.SetStyle(YAML::EmitterStyle::Flow);

    YAML::Node orientation_R;
    for (int i = 0; i < 3; ++i)
    {
        YAML::Node row;
        for (int j = 0; j < 3; ++j) row.push_back(transform_matrix_R(i, j));
        row.SetStyle(YAML::EmitterStyle::Flow);
        orientation_R.push_back(row);
    }

    root["target_pose_R"]["position"] = position_R;
    root["target_pose_R"]["orientation"] = orientation_R;

    // ✅ target_pose_L
    YAML::Node position_L;
    position_L.push_back(transform_matrix_L(0, 3));
    position_L.push_back(transform_matrix_L(1, 3));
    position_L.push_back(transform_matrix_L(2, 3));
    position_L.SetStyle(YAML::EmitterStyle::Flow);

    YAML::Node orientation_L;
    for (int i = 0; i < 3; ++i)
    {
        YAML::Node row;
        for (int j = 0; j < 3; ++j) row.push_back(transform_matrix_L(i, j));
        row.SetStyle(YAML::EmitterStyle::Flow);
        orientation_L.push_back(row);
    }

    root["target_pose_L"]["position"] = position_L;
    root["target_pose_L"]["orientation"] = orientation_L;

    // ✅ 写回文件
    std::ofstream file_out(filename, std::ios::out | std::ios::trunc);
    if (file_out.is_open())
    {
        YAML::Emitter out;
        out.SetIndent(2);  // 可选美化缩进
        out << root;
        file_out << out.c_str();
        file_out.close();
        ROS_INFO_STREAM("Saved target_pose_R and target_pose_L to " << filename);
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
        tf::StampedTransform tf_world_obj, tf_world_obj_1, tf_world_obj_2;
        tf::StampedTransform tf_link2_0_flan2, tf_link3_0_flan3;
        tf::StampedTransform tf_world_cube_l, tf_world_cube_r;
        tf::StampedTransform tf_base_link2_0, tf_base_link3_0;
        tf::StampedTransform tf_base_camera_link;
        try
        {
            listener.lookupTransform("world", "dummy_point3", ros::Time(0), transform);
            listener.lookupTransform("world", "dummy_point4", ros::Time(0), transform2);
            listener.lookupTransform("world", "flan1", ros::Time(0), tf_world_flan1);
            listener.lookupTransform("world", "flan4", ros::Time(0), tf_world_flan4);
            listener.lookupTransform("base_link", "Link1_0", ros::Time(0), tf_base_link1_0);
            listener.lookupTransform("base_link", "Link4_0", ros::Time(0), tf_base_link4_0);
            listener.lookupTransform("base_link", "Link2_0", ros::Time(0), tf_base_link2_0);
            listener.lookupTransform("base_link", "Link3_0", ros::Time(0), tf_base_link3_0);
            listener.lookupTransform("world", "object", ros::Time(0), tf_world_obj);
            listener.lookupTransform("world", "object1", ros::Time(0), tf_world_obj_1);
            listener.lookupTransform("world", "object2", ros::Time(0), tf_world_obj_2);
            listener.lookupTransform("Link2_0", "flan2", ros::Time(0), tf_link2_0_flan2);
            listener.lookupTransform("Link3_0", "flan3", ros::Time(0), tf_link3_0_flan3);
            listener.lookupTransform("world", "cube_l", ros::Time(0), tf_world_cube_l);
            listener.lookupTransform("world", "cube_r", ros::Time(0), tf_world_cube_r);
            listener.lookupTransform("base_link", "camera_link", ros::Time(0), tf_base_camera_link);

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
            Eigen::Matrix4f tf_mat_world_obj_2 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_link2_0_flan2 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_link3_0_flan3 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_world_cube_l = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_world_cube_r = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_mat_base_camera_link = Eigen::Matrix4f::Identity();

            tf::Matrix3x3 rotation_matrix(transform.getRotation());
            tf::Matrix3x3 rotation_matrix2(transform2.getRotation());
            tf::Matrix3x3 rot_mat_world_flan1(tf_world_flan1.getRotation());
            tf::Matrix3x3 rot_mat_world_flan4(tf_world_flan4.getRotation());
            tf::Matrix3x3 rot_mat_base_link1_0(tf_base_link1_0.getRotation());
            tf::Matrix3x3 rot_mat_base_link4_0(tf_base_link4_0.getRotation());
            tf::Matrix3x3 rot_mat_base_link2_0(tf_base_link2_0.getRotation());
            tf::Matrix3x3 rot_mat_base_link3_0(tf_base_link3_0.getRotation());
            tf::Matrix3x3 rot_mat_world_obj(tf_world_obj.getRotation());
            tf::Matrix3x3 rot_mat_world_obj_1(tf_world_obj_1.getRotation());
            tf::Matrix3x3 rot_mat_world_obj_2(tf_world_obj_2.getRotation());
            tf::Matrix3x3 rot_mat_link2_0_flan2(tf_link2_0_flan2.getRotation());
            tf::Matrix3x3 rot_mat_link3_0_flan3(tf_link3_0_flan3.getRotation());
            tf::Matrix3x3 rot_mat_world_cube_l(tf_world_cube_l.getRotation());
            tf::Matrix3x3 rot_mat_world_cube_r(tf_world_cube_r.getRotation());
            tf::Matrix3x3 rot_mat_base_camera_link(tf_base_camera_link.getRotation());

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
                    tf_mat_world_obj_2(i, j) = rot_mat_world_obj_2[i][j];
                    tf_mat_link2_0_flan2(i, j) = rot_mat_link2_0_flan2[i][j];
                    tf_mat_link3_0_flan3(i, j) = rot_mat_link3_0_flan3[i][j];
                    tf_mat_world_cube_l(i, j) = rot_mat_world_cube_l[i][j];
                    tf_mat_world_cube_r(i, j) = rot_mat_world_cube_r[i][j];
                    tf_mat_base_camera_link(i, j) = rot_mat_base_camera_link[i][j];
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

            tf_mat_base_link2_0(0, 3) = tf_base_link2_0.getOrigin().x();
            tf_mat_base_link2_0(1, 3) = tf_base_link2_0.getOrigin().y();
            tf_mat_base_link2_0(2, 3) = tf_base_link2_0.getOrigin().z();

            tf_mat_base_link3_0(0, 3) = tf_base_link3_0.getOrigin().x();
            tf_mat_base_link3_0(1, 3) = tf_base_link3_0.getOrigin().y();
            tf_mat_base_link3_0(2, 3) = tf_base_link3_0.getOrigin().z();

            tf_mat_world_obj(0, 3) = tf_world_obj.getOrigin().x();
            tf_mat_world_obj(1, 3) = tf_world_obj.getOrigin().y();
            tf_mat_world_obj(2, 3) = tf_world_obj.getOrigin().z();

            tf_mat_world_obj_1(0, 3) = tf_world_obj_1.getOrigin().x();
            tf_mat_world_obj_1(1, 3) = tf_world_obj_1.getOrigin().y();
            tf_mat_world_obj_1(2, 3) = tf_world_obj_1.getOrigin().z();

            tf_mat_world_obj_2(0, 3) = tf_world_obj_2.getOrigin().x();
            tf_mat_world_obj_2(1, 3) = tf_world_obj_2.getOrigin().y();
            tf_mat_world_obj_2(2, 3) = tf_world_obj_2.getOrigin().z();

            tf_mat_link2_0_flan2(0, 3) = tf_link2_0_flan2.getOrigin().x();
            tf_mat_link2_0_flan2(1, 3) = tf_link2_0_flan2.getOrigin().y();
            tf_mat_link2_0_flan2(2, 3) = tf_link2_0_flan2.getOrigin().z();

            tf_mat_link3_0_flan3(0, 3) = tf_link3_0_flan3.getOrigin().x();
            tf_mat_link3_0_flan3(1, 3) = tf_link3_0_flan3.getOrigin().y();
            tf_mat_link3_0_flan3(2, 3) = tf_link3_0_flan3.getOrigin().z();

            tf_mat_world_cube_l(0, 3) = tf_world_cube_l.getOrigin().x();
            tf_mat_world_cube_l(1, 3) = tf_world_cube_l.getOrigin().y();
            tf_mat_world_cube_l(2, 3) = tf_world_cube_l.getOrigin().z();

            tf_mat_world_cube_r(0, 3) = tf_world_cube_r.getOrigin().x();
            tf_mat_world_cube_r(1, 3) = tf_world_cube_r.getOrigin().y();
            tf_mat_world_cube_r(2, 3) = tf_world_cube_r.getOrigin().z();

            tf_mat_base_camera_link(0, 3) = tf_base_camera_link.getOrigin().x();
            tf_mat_base_camera_link(1, 3) = tf_base_camera_link.getOrigin().y();
            tf_mat_base_camera_link(2, 3) = tf_base_camera_link.getOrigin().z();

            saveTransformToYAML(yaml_path, transform_matrix, transform_matrix2);
            saveTransformToYAML(path_tf_double_arm_float, transform_matrix, transform_matrix2);
            saveTFToYAML(path_tf_using, tf_mat_world_flan1, "tf_mat_world_flan1");
            saveTFToYAML(path_tf_using, tf_mat_world_flan4, "tf_mat_world_flan4");
            saveTFToYAML(path_tf_using, tf_mat_base_link1_0, "tf_mat_base_link1_0");
            saveTFToYAML(path_tf_using, tf_mat_base_link4_0, "tf_mat_base_link4_0");
            saveTFToYAML(path_tf_using, tf_mat_base_link2_0, "tf_mat_base_link2_0");
            saveTFToYAML(path_tf_using, tf_mat_base_link3_0, "tf_mat_base_link3_0");
            saveTFToYAML(path_tf_using, tf_mat_link2_0_flan2, "tf_mat_link2_0_flan2");
            saveTFToYAML(path_tf_using, tf_mat_link3_0_flan3, "tf_mat_link3_0_flan3");
            saveTFToYAML(path_tf_using, tf_mat_base_camera_link, "tf_mat_base_camera_link");

            saveTFToYAML(path_tf_obj, tf_mat_base_link1_0, "tf_mat_base_link1_0");
            saveTFToYAML(path_tf_obj, tf_mat_base_link2_0, "tf_mat_base_link2_0");
            saveTFToYAML(path_tf_obj, tf_mat_base_link3_0, "tf_mat_base_link3_0");
            saveTFToYAML(path_tf_obj, tf_mat_base_link4_0, "tf_mat_base_link4_0");
            saveTFToYAML(path_tf_obj, tf_mat_world_obj, "tf_mat_world_obj");
            saveTFToYAML(path_tf_obj, tf_mat_world_obj_1, "tf_mat_world_obj_1");
            saveTFToYAML(path_tf_obj, tf_mat_world_obj_2, "tf_mat_world_obj_2");
            saveTFToYAML(path_tf_obj, tf_mat_world_cube_l, "tf_mat_world_cube_l");
            saveTFToYAML(path_tf_obj, tf_mat_world_cube_r, "tf_mat_world_cube_r");
            saveTFToYAML(path_tf_obj, tf_mat_base_camera_link, "tf_mat_base_camera_link");
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        rate.sleep();
    }
    return 0;
}