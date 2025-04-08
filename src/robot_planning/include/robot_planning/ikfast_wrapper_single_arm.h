#ifndef ROBOT_PLANNING_IKFAST_WRAPPER_SINGLE_ARM_H
#define ROBOT_PLANNING_IKFAST_WRAPPER_SINGLE_ARM_H

#include <vector>

namespace robots
{

    class Kinematics
    {
    public:
        Kinematics();
        ~Kinematics();

        // 逆运动学求解器
        std::vector<float> inverse(std::vector<float>& ee_pose);

        // 正向运动学求解器
        std::vector<float> forward(std::vector<float>& joint_config);

        // 机器人关节数量
        int num_of_joints;
        int num_free_parameters;
    };

}  // namespace robots

#endif  // ROBOT_PLANNING_IKFAST_WRAPPER_SINGLE_ARM_H
