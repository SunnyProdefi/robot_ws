#include "ikfast_single_arm.cpp"                       // IKFast 自动生成的文件
#include "robot_planning/ikfast_wrapper_single_arm.h"  // 类声明头文件

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>

#define IK_VERSION 61

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal
#else
#define IKREAL_TYPE IKReal
#endif

namespace robots
{

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

    Kinematics::~Kinematics() {}

    std::vector<float> Kinematics::forward(std::vector<float>& joint_config)
    {
        IKREAL_TYPE eerot[9], eetrans[3];
        std::vector<float> ee_pose;

        if (joint_config.size() != num_of_joints)
        {
            printf(
                "\nError: (forward kinematics) expects vector of %d values describing "
                "joint angles (in radians).\n\n",
                num_of_joints);
            return ee_pose;
        }

        // Convert joint configuration to array
        IKREAL_TYPE joints[num_of_joints];
        for (unsigned int i = 0; i < num_of_joints; i++)
        {
            joints[i] = joint_config[i];
        }

#if IK_VERSION > 54
        ComputeFk(joints, eetrans, eerot);  // for IKFast 56,61
#else
        fk(joints, eetrans, eerot);  // for IKFast 54
#endif

        // Extract end-effector pose
        for (unsigned int i = 0; i < 3; i++)
        {
            ee_pose.push_back(eerot[i * 3 + 0]);
            ee_pose.push_back(eerot[i * 3 + 1]);
            ee_pose.push_back(eerot[i * 3 + 2]);
            ee_pose.push_back(eetrans[i]);
        }

        return ee_pose;
    }

    std::vector<float> Kinematics::inverse(std::vector<float>& ee_pose)
    {
        IKREAL_TYPE eerot[9], eetrans[3];
        std::vector<float> joint_configs;

        if (ee_pose.size() == 7)
        {  // Quaternion representation
#if IK_VERSION > 54
            IkSolutionList<IKREAL_TYPE> solutions;
#else
            std::vector<IKSolution> vsolutions;
#endif
            std::vector<IKREAL_TYPE> vfree(num_free_parameters);

            // Convert translation and quaternion to matrix
            eetrans[0] = ee_pose[0];
            eetrans[1] = ee_pose[1];
            eetrans[2] = ee_pose[2];
            double qw = ee_pose[3], qx = ee_pose[4], qy = ee_pose[5], qz = ee_pose[6];
            const double n = 1.0f / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
            qw *= n;
            qx *= n;
            qy *= n;
            qz *= n;

            // Calculate rotation matrix from quaternion
            eerot[0] = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
            eerot[1] = 2.0f * qx * qy - 2.0f * qz * qw;
            eerot[2] = 2.0f * qx * qz + 2.0f * qy * qw;
            eerot[3] = 2.0f * qx * qy + 2.0f * qz * qw;
            eerot[4] = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
            eerot[5] = 2.0f * qy * qz - 2.0f * qx * qw;
            eerot[6] = 2.0f * qx * qz - 2.0f * qy * qw;
            eerot[7] = 2.0f * qy * qz + 2.0f * qx * qw;
            eerot[8] = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;

            // Compute inverse kinematics
#if IK_VERSION > 54
            bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            bool bSuccess = ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif

            if (!bSuccess)
            {
                fprintf(stderr, "Error: (inverse kinematics) failed to get ik solution\n");
                return joint_configs;
            }

            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
            std::vector<IKREAL_TYPE> solvalues(num_of_joints);
            for (unsigned int i = 0; i < num_of_solutions; ++i)
            {
#if IK_VERSION > 54
                const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                sol.GetSolution(&solvalues[0], vfree.size() > 0 ? &vfree[0] : NULL);
#else
                vsolutions[i].GetSolution(&solvalues[0], vfree.size() > 0 ? &vfree[0] : NULL);
#endif
                for (unsigned int j = 0; j < solvalues.size(); ++j)
                {
                    joint_configs.push_back(solvalues[j]);
                }
            }
        }
        else if (ee_pose.size() == 12)
        {  // Matrix representation
           // Handle the case for 12-element transformation matrix similarly to quaternion case
           // Similar process as above to populate eerot and eetrans
        }
        else
        {
            printf("\nError: (inverse kinematics) invalid pose format\n");
            return joint_configs;
        }

        return joint_configs;
    }

}  // namespace robots
