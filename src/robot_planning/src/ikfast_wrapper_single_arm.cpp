#define IKFAST_HAS_LIBRARY
#ifndef IKFAST_NO_MAIN
#define IKFAST_NO_MAIN
#endif

#define IK_VERSION 61
#include "ikfast_single_arm.cpp"                       // IKFast 自动生成的文件
#include "robot_planning/ikfast_wrapper_single_arm.h"  // 类声明头文件

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal
#else
#define IKREAL_TYPE IKReal
#endif

namespace robots
{

    // 构造函数
    Kinematics::Kinematics()
    {
#if IK_VERSION > 54
        num_of_joints = GetNumJoints();
        num_free_parameters = GetNumFreeParameters();
#else
        num_of_joints = getNumJoints();
        num_free_parameters = getNumFreeParameters();
#endif
    }

    // 析构函数
    Kinematics::~Kinematics() {}

    // 正向运动学
    std::vector<float> Kinematics::forward(const std::vector<float>& joint_config)
    {
        IKREAL_TYPE eerot[9], eetrans[3];
        std::vector<float> ee_pose;

        if (joint_config.size() != num_of_joints)
        {
            printf("[IKFast] forward(): joint_config size mismatch\n");
            return ee_pose;
        }

        IKREAL_TYPE joints[num_of_joints];
        for (unsigned int i = 0; i < num_of_joints; i++) joints[i] = joint_config[i];

        ComputeFk(joints, eetrans, eerot);

        for (int i = 0; i < 3; ++i)
        {
            ee_pose.push_back(eerot[i * 3 + 0]);
            ee_pose.push_back(eerot[i * 3 + 1]);
            ee_pose.push_back(eerot[i * 3 + 2]);
            ee_pose.push_back(eetrans[i]);
        }

        return ee_pose;
    }

    // 逆向运动学
    std::vector<float> Kinematics::inverse(const std::vector<float>& ee_pose)
    {
        IKREAL_TYPE eerot[9], eetrans[3];
        std::vector<float> joint_configs;
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);

#if IK_VERSION > 54
        IkSolutionList<IKREAL_TYPE> solutions;
#else
        std::vector<IKSolution> vsolutions;
#endif

        if (ee_pose.size() == 7)
        {
            eetrans[0] = ee_pose[0];
            eetrans[1] = ee_pose[1];
            eetrans[2] = ee_pose[2];

            double qw = ee_pose[3], qx = ee_pose[4], qy = ee_pose[5], qz = ee_pose[6];
            double n = 1.0 / std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
            qw *= n;
            qx *= n;
            qy *= n;
            qz *= n;

            eerot[0] = 1 - 2 * qy * qy - 2 * qz * qz;
            eerot[1] = 2 * qx * qy - 2 * qz * qw;
            eerot[2] = 2 * qx * qz + 2 * qy * qw;
            eerot[3] = 2 * qx * qy + 2 * qz * qw;
            eerot[4] = 1 - 2 * qx * qx - 2 * qz * qz;
            eerot[5] = 2 * qy * qz - 2 * qx * qw;
            eerot[6] = 2 * qx * qz - 2 * qy * qw;
            eerot[7] = 2 * qy * qz + 2 * qx * qw;
            eerot[8] = 1 - 2 * qx * qx - 2 * qy * qy;
        }
        else if (ee_pose.size() == 12)
        {
            eerot[0] = ee_pose[0];
            eerot[1] = ee_pose[1];
            eerot[2] = ee_pose[2];
            eetrans[0] = ee_pose[3];
            eerot[3] = ee_pose[4];
            eerot[4] = ee_pose[5];
            eerot[5] = ee_pose[6];
            eetrans[1] = ee_pose[7];
            eerot[6] = ee_pose[8];
            eerot[7] = ee_pose[9];
            eerot[8] = ee_pose[10];
            eetrans[2] = ee_pose[11];
        }
        else
        {
            fprintf(stderr, "[IKFast] inverse(): ee_pose size must be 7 or 12.\n");
            return joint_configs;
        }

#if IK_VERSION > 54
        bool success = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
        if (!success)
            return joint_configs;
        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
        for (size_t i = 0; i < solutions.GetNumSolutions(); ++i)
        {
            const auto& sol = solutions.GetSolution(i);
            sol.GetSolution(&solvalues[0], vfree.size() > 0 ? &vfree[0] : NULL);
            joint_configs.insert(joint_configs.end(), solvalues.begin(), solvalues.end());
        }
#else
        bool success = ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
        if (!success)
            return joint_configs;
        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
        for (size_t i = 0; i < vsolutions.size(); ++i)
        {
            vsolutions[i].GetSolution(&solvalues[0], vfree.size() > 0 ? &vfree[0] : NULL);
            joint_configs.insert(joint_configs.end(), solvalues.begin(), solvalues.end());
        }
#endif

        return joint_configs;
    }

}  // namespace robots
