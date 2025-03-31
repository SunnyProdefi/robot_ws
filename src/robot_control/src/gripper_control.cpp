#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "gripper_driver.h"

// 创建夹爪实例
std::array<GripperDriver, 4> grippers = {GripperDriver("/dev/gripper1", CANUSB_SPEED_1000000, CANUSB_TTY_BAUD_RATE_DEFAULT, "01"), GripperDriver("/dev/gripper2", CANUSB_SPEED_1000000, CANUSB_TTY_BAUD_RATE_DEFAULT, "02"),
                                         GripperDriver("/dev/gripper3", CANUSB_SPEED_1000000, CANUSB_TTY_BAUD_RATE_DEFAULT, "03"), GripperDriver("/dev/gripper4", CANUSB_SPEED_1000000, CANUSB_TTY_BAUD_RATE_DEFAULT, "04")};

ros::Publisher gripper_state_pub;  // 全局定义

void gripperCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != grippers.size())
    {
        ROS_WARN("Received %lu gripper commands, but have %lu grippers", msg->data.size(), grippers.size());
        return;
    }

    std_msgs::Float64MultiArray state_msg;
    state_msg.data.resize(grippers.size());

    for (size_t i = 0; i < grippers.size(); ++i)
    {
        double pos_cmd = msg->data[i];
        double pos_actual = grippers[i].gripper_control(pos_cmd, 1.0, 1.0, 1.0, 1.0);
        state_msg.data[i] = pos_actual;
    }

    gripper_state_pub.publish(state_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_controller_node");
    ros::NodeHandle nh;
    gripper_state_pub = nh.advertise<std_msgs::Float64MultiArray>("/gripper_state", 10);
    // 初始化夹爪
    for (size_t i = 0; i < grippers.size(); ++i)
    {
        if (grippers[i].Init() < 0)
        {
            std::cerr << "Failed to initialize gripper " << (i + 1) << std::endl;
            return EXIT_FAILURE;
        }
    }

    ros::Subscriber sub = nh.subscribe("/gripper_command", 10, gripperCallback);

    ros::spin();

    return 0;
}
