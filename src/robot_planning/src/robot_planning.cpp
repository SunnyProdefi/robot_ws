#include <ros/ros.h>
#include "robot_planning/PlanPath.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

bool planCallback(robot_planning::PlanPath::Request& req, robot_planning::PlanPath::Response& res)
{
    ROS_INFO("Received planning request...");

    // 示例轨迹（可替换为实际算法）
    std::vector<std::vector<double>> trajectory = {{0.0, 0.1, 0.2, 0.3}, {0.1, 0.2, 0.3, 0.4}, {0.2, 0.3, 0.4, 0.5}};

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "joint_trajectory" << YAML::Value << YAML::BeginSeq;
    for (const auto& point : trajectory) out << YAML::Flow << point;
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout("/home/prodefi/planned_trajectory.yaml");
    fout << out.c_str();
    fout.close();

    res.success = true;
    res.message = "Trajectory saved to planned_trajectory.yaml";
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("plan_path", planCallback);
    ROS_INFO("Path planner node ready.");
    ros::spin();
    return 0;
}
