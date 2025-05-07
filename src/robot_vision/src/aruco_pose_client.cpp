#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_pose_client");
    ros::NodeHandle nh;

    ros::service::waitForService("/detect_aruco_and_save");
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/detect_aruco_and_save");

    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("Service call successful: %s", srv.response.message.c_str());

            std::string pkg_path = ros::package::getPath("robot_vision");
            std::string yaml_path = pkg_path + "/config/aruco_pose.yaml";

            try
            {
                YAML::Node data = YAML::LoadFile(yaml_path);
                const auto& tf_node = data["tf_mat_camera_obj"];
                if (!tf_node)
                {
                    ROS_ERROR("Missing key: tf_mat_camera_obj");
                    return 1;
                }

                const auto& pose = tf_node["target_pose"];
                const auto& pos = pose["position"];
                const auto& ori = pose["orientation"];

                std::cout << "\n=== ArUco Pose ===" << std::endl;

                std::cout << "Position: ";
                for (const auto& val : pos) std::cout << val.as<double>() << " ";
                std::cout << std::endl;

                std::cout << "Orientation Matrix:" << std::endl;
                for (const auto& row : ori)
                {
                    for (const auto& val : row) std::cout << val.as<double>() << " ";
                    std::cout << std::endl;
                }
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("Failed to read YAML file: %s", e.what());
            }
        }
        else
        {
            ROS_WARN("Service Failure: %s", srv.response.message.c_str());
        }
    }
    else
    {
        ROS_ERROR("Service call failed");
    }

    return 0;
}
