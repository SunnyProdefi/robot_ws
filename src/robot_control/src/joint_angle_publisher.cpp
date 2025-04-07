#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <array>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

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

sensor_msgs::JointState joint_msg;

void motorStateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // 分支1
    joint_msg.position[JOINT1_1] = msg->data[0];
    joint_msg.position[JOINT1_2] = msg->data[1];
    joint_msg.position[JOINT1_3] = msg->data[2];
    joint_msg.position[JOINT1_4] = msg->data[3];
    joint_msg.position[JOINT1_5] = msg->data[4];
    joint_msg.position[JOINT1_6] = msg->data[5];
    // 分支1的夹爪
    joint_msg.position[LINK1_FINGER] = 1.0 - msg->data[6];

    // 分支2
    joint_msg.position[JOINT2_1] = msg->data[7];
    joint_msg.position[JOINT2_2] = msg->data[8];
    joint_msg.position[JOINT2_3] = msg->data[9];
    joint_msg.position[JOINT2_4] = msg->data[10];
    joint_msg.position[JOINT2_5] = msg->data[11];
    joint_msg.position[JOINT2_6] = msg->data[12];
    // 分支2的夹爪
    joint_msg.position[LINK2_FINGER] = 1.0 - msg->data[13];

    // 分支3
    joint_msg.position[JOINT3_1] = msg->data[14];
    joint_msg.position[JOINT3_2] = msg->data[15];
    joint_msg.position[JOINT3_3] = msg->data[16];
    joint_msg.position[JOINT3_4] = msg->data[17];
    joint_msg.position[JOINT3_5] = msg->data[18];
    joint_msg.position[JOINT3_6] = msg->data[19];
    // 分支3的夹爪
    joint_msg.position[LINK3_FINGER] = 1.0 - msg->data[20];

    // 分支4
    joint_msg.position[JOINT4_1] = msg->data[21];
    joint_msg.position[JOINT4_2] = msg->data[22];
    joint_msg.position[JOINT4_3] = msg->data[23];
    joint_msg.position[JOINT4_4] = msg->data[24];
    joint_msg.position[JOINT4_5] = msg->data[25];
    joint_msg.position[JOINT4_6] = msg->data[26];
    // 分支4的夹爪
    joint_msg.position[LINK4_FINGER] = 1.0 - msg->data[27];
}

geometry_msgs::Pose latest_base_pose;
bool base_pose_received = false;

void floatBaseStateCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    latest_base_pose = *msg;
    base_pose_received = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_angle_publisher_idmap");
    ros::NodeHandle nh;

    // 初始化 TF 广播器
    static tf::TransformBroadcaster tf_broadcaster;

    // 创建发布器
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // 订阅motor_state话题
    ros::Subscriber motor_state_sub = nh.subscribe<std_msgs::Float64MultiArray>("/motor_state", 10, motorStateCallback);

    // 订阅floating_base_state话题
    ros::Subscriber floating_base_state_sub = nh.subscribe<geometry_msgs::Pose>("/floating_base_state", 10, floatBaseStateCallback);

    joint_msg.name.resize(JOINT_COUNT);
    joint_msg.position.resize(JOINT_COUNT);
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        joint_msg.name[i] = JOINT_NAMES[i];
        joint_msg.position[i] = 0.0;
    }

    joint_msg.position[JOINT1_0] = 1.5708;
    joint_msg.position[JOINT2_0] = 1.5708;
    joint_msg.position[JOINT3_0] = 1.5708;
    joint_msg.position[JOINT4_0] = 1.5708;

    joint_msg.position[JOINT1_PLATLINK] = -0.847454;
    joint_msg.position[JOINT1_PLATLINK2] = -2.41825;
    joint_msg.position[JOINT2_PLATLINK] = -0.847454;
    joint_msg.position[JOINT2_PLATLINK2] = -2.41825;
    joint_msg.position[JOINT3_PLATLINK] = -0.847454;
    joint_msg.position[JOINT3_PLATLINK2] = -2.41825;
    joint_msg.position[JOINT4_PLATLINK] = -0.847454;
    joint_msg.position[JOINT4_PLATLINK2] = -2.41825;

    joint_msg.position[JOINT_PLATFORM] = 0.077888;

    ros::Rate loop_rate(10);  // 10Hz

    while (ros::ok())
    {
        joint_msg.header.stamp = ros::Time::now();
        joint_pub.publish(joint_msg);

        // 发布 TF
        if (base_pose_received)
        {
            tf::Vector3 translation(latest_base_pose.position.x, latest_base_pose.position.y, latest_base_pose.position.z);

            tf::Quaternion rotation(latest_base_pose.orientation.x, latest_base_pose.orientation.y, latest_base_pose.orientation.z, latest_base_pose.orientation.w);

            tf::Transform transform;
            transform.setOrigin(translation);
            transform.setRotation(rotation);

            geometry_msgs::TransformStamped t;
            t.header.stamp = ros::Time::now();
            t.header.frame_id = "world";
            t.child_frame_id = "base_link";
            t.transform.translation.x = translation.x();
            t.transform.translation.y = translation.y();
            t.transform.translation.z = translation.z();
            t.transform.rotation.x = rotation.x();
            t.transform.rotation.y = rotation.y();
            t.transform.rotation.z = rotation.z();
            t.transform.rotation.w = rotation.w();

            tf_broadcaster.sendTransform(t);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
