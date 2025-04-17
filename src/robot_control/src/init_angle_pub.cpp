#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <array>

// 用枚举来给 41 个关节分配索引 ID，方便快速访问
enum JointID : int
{
    // 先放平台关节
    JOINT_PLATFORM = 0,

    // Branch 1
    JOINT1_0,
    JOINT1_1,
    JOINT1_2,
    JOINT1_3,
    JOINT1_4,
    JOINT1_5,
    JOINT1_6,
    LINK1_FINGER,

    // Branch 2
    JOINT2_0,
    JOINT2_1,
    JOINT2_2,
    JOINT2_3,
    JOINT2_4,
    JOINT2_5,
    JOINT2_6,
    LINK2_FINGER,

    // Branch 3
    JOINT3_0,
    JOINT3_1,
    JOINT3_2,
    JOINT3_3,
    JOINT3_4,
    JOINT3_5,
    JOINT3_6,
    LINK3_FINGER,

    // Branch 4
    JOINT4_0,
    JOINT4_1,
    JOINT4_2,
    JOINT4_3,
    JOINT4_4,
    JOINT4_5,
    JOINT4_6,
    LINK4_FINGER,

    // Platlink / Platlink2
    JOINT1_PLATLINK,
    JOINT1_PLATLINK2,
    JOINT2_PLATLINK,
    JOINT2_PLATLINK2,
    JOINT3_PLATLINK,
    JOINT3_PLATLINK2,
    JOINT4_PLATLINK,
    JOINT4_PLATLINK2,

    // 用来标记总数量，必须放在最后
    JOINT_COUNT
};

// 建立一个数组，把上面枚举的每个 ID 与具体关节名称一一对应
static const std::array<std::string, JOINT_COUNT> JOINT_NAMES = {
    /* JOINT_PLATFORM */ "Joint_platform",

    /* JOINT1_0 */ "Joint1_0",
    /* JOINT1_1 */ "Joint1_1",
    /* JOINT1_2 */ "Joint1_2",
    /* JOINT1_3 */ "Joint1_3",
    /* JOINT1_4 */ "Joint1_4",
    /* JOINT1_5 */ "Joint1_5",
    /* JOINT1_6 */ "Joint1_6",
    /* LINK1_FINGER */ "Link1_finger_joint",

    /* JOINT2_0 */ "Joint2_0",
    /* JOINT2_1 */ "Joint2_1",
    /* JOINT2_2 */ "Joint2_2",
    /* JOINT2_3 */ "Joint2_3",
    /* JOINT2_4 */ "Joint2_4",
    /* JOINT2_5 */ "Joint2_5",
    /* JOINT2_6 */ "Joint2_6",
    /* LINK2_FINGER */ "Link2_finger_joint",

    /* JOINT3_0 */ "Joint3_0",
    /* JOINT3_1 */ "Joint3_1",
    /* JOINT3_2 */ "Joint3_2",
    /* JOINT3_3 */ "Joint3_3",
    /* JOINT3_4 */ "Joint3_4",
    /* JOINT3_5 */ "Joint3_5",
    /* JOINT3_6 */ "Joint3_6",
    /* LINK3_FINGER */ "Link3_finger_joint",

    /* JOINT4_0 */ "Joint4_0",
    /* JOINT4_1 */ "Joint4_1",
    /* JOINT4_2 */ "Joint4_2",
    /* JOINT4_3 */ "Joint4_3",
    /* JOINT4_4 */ "Joint4_4",
    /* JOINT4_5 */ "Joint4_5",
    /* JOINT4_6 */ "Joint4_6",
    /* LINK4_FINGER */ "Link4_finger_joint",

    /* JOINT1_PLATLINK */ "Joint1_platlink",
    /* JOINT1_PLATLINK2 */ "Joint1_platlink2",
    /* JOINT2_PLATLINK */ "Joint2_platlink",
    /* JOINT2_PLATLINK2 */ "Joint2_platlink2",
    /* JOINT3_PLATLINK */ "Joint3_platlink",
    /* JOINT3_PLATLINK2 */ "Joint3_platlink2",
    /* JOINT4_PLATLINK */ "Joint4_platlink",
    /* JOINT4_PLATLINK2 */ "Joint4_platlink2",
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init_angle_pub");
    ros::NodeHandle nh;

    // 创建发布器
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // 准备 JointState 消息：大小 = 41
    sensor_msgs::JointState joint_msg;
    joint_msg.name.resize(JOINT_COUNT);
    joint_msg.position.resize(JOINT_COUNT);

    // 将关节名一次性拷贝过去
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        joint_msg.name[i] = JOINT_NAMES[i];
        joint_msg.position[i] = 0.0;  // 初始设为 0
    }

    joint_msg.position[JOINT1_0] = 1.5708;  // Body_displace
    joint_msg.position[JOINT2_0] = 1.5708;  // Body_displace
    joint_msg.position[JOINT3_0] = 1.5708;  // Body_displace
    joint_msg.position[JOINT4_0] = 1.5708;  // Body_displace

    // joint_msg.position[JOINT1_1] = 1.597727;
    // joint_msg.position[JOINT1_2] = 0.295055;
    // joint_msg.position[JOINT1_3] = 2.156446;
    // joint_msg.position[JOINT1_4] = 0.040097;
    // joint_msg.position[JOINT1_5] = 0.494959;
    // joint_msg.position[JOINT1_6] = 3.125349;

    joint_msg.position[JOINT1_1] = 1.597727;
    joint_msg.position[JOINT1_2] = 0.295055;
    joint_msg.position[JOINT1_3] = 2.156446;
    joint_msg.position[JOINT1_4] = 3.101495;
    joint_msg.position[JOINT1_5] = -0.494894;
    joint_msg.position[JOINT1_6] = -0.016370;

    joint_msg.position[JOINT4_1] = -1.597727;
    joint_msg.position[JOINT4_2] = 0.295055;
    joint_msg.position[JOINT4_3] = 2.156446;
    joint_msg.position[JOINT4_4] = -0.040097;
    joint_msg.position[JOINT4_5] = 0.494959;
    joint_msg.position[JOINT4_6] = 0.016244;

    joint_msg.position[JOINT2_1] = -1.23754;
    joint_msg.position[JOINT2_2] = -2.83752;
    joint_msg.position[JOINT2_3] = -1.99311;
    joint_msg.position[JOINT2_4] = -1.31756;
    joint_msg.position[JOINT2_5] = -1.35178;
    joint_msg.position[JOINT2_6] = -0.0869968;

    joint_msg.position[JOINT3_1] = 0.962226;
    joint_msg.position[JOINT3_2] = -2.83752;
    joint_msg.position[JOINT3_3] = -1.99311;
    joint_msg.position[JOINT3_4] = 1.09056;
    joint_msg.position[JOINT3_5] = -1.18132;
    joint_msg.position[JOINT3_6] = -2.98608;

    joint_msg.position[JOINT1_PLATLINK] = -0.847454;
    joint_msg.position[JOINT1_PLATLINK2] = -2.41825;
    joint_msg.position[JOINT2_PLATLINK] = -0.847454;
    joint_msg.position[JOINT2_PLATLINK2] = -2.41825;
    joint_msg.position[JOINT3_PLATLINK] = -0.847454;
    joint_msg.position[JOINT3_PLATLINK2] = -2.41825;
    joint_msg.position[JOINT4_PLATLINK] = -0.847454;
    joint_msg.position[JOINT4_PLATLINK2] = -2.41825;

    joint_msg.position[LINK1_FINGER] = 0.5;
    joint_msg.position[LINK2_FINGER] = 0.0;
    joint_msg.position[LINK3_FINGER] = 0.0;
    joint_msg.position[LINK4_FINGER] = 0.5;

    joint_msg.position[JOINT_PLATFORM] = 0.077888;

    ros::Rate loop_rate(10);  // 10Hz
    double phase = 0.0;

    while (ros::ok())
    {
        joint_msg.header.stamp = ros::Time::now();
        joint_pub.publish(joint_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
