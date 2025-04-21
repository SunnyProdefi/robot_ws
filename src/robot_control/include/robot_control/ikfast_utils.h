#include <cmath>
#include <vector>

// 欧几里得距离计算
inline float calculateDistance(const std::vector<float>& a, const std::vector<float>& b)
{
    float distance = 0.0f;
    for (size_t i = 0; i < a.size(); ++i)
    {
        float diff = a[i] - b[i];
        distance += diff * diff;
    }
    return std::sqrt(distance);
}

// 归一化 angle 到以 reference 为中心的最近等效角度
inline float normalizeAngleToNearest(float angle, float reference)
{
    const float TWO_PI = 2.0f * static_cast<float>(M_PI);
    float diff = angle - reference;

    while (diff > M_PI) diff -= TWO_PI;
    while (diff < -M_PI) diff += TWO_PI;

    return reference + diff;
}

// 计算所有解，并找到最优解（并对第4个关节归一化后替换原数据）
inline std::pair<std::vector<std::vector<float>>, std::vector<float>> findAllSolutions(std::vector<float>& ik_results,       // 输入所有 IK 解的一维向量
                                                                                const std::vector<float>& initial_q,  // 初始关节角
                                                                                size_t num_joints                     // 关节数量
)
{
    std::vector<std::vector<float>> all_solutions;
    std::vector<float> closest_solution;
    float min_distance = std::numeric_limits<float>::max();

    // 定义关节限位
    std::vector<std::pair<float, float>> joint_limits = {
        {-2.3562, 4.7124},  // Joint2_1
        {-2.0944, 1.5708},  // Joint2_2
        {-2.5307, 2.5307},  // Joint2_3
        {-3.1416, 3.1416},  // Joint2_4
        {-1.9199, 1.9199},  // Joint2_5
        {-3.1416, 3.1416}   // Joint2_6
    };

    for (size_t i = 0; i < ik_results.size(); i += num_joints)
    {
        std::vector<float> solution(ik_results.begin() + i, ik_results.begin() + i + num_joints);

        // 检查是否在限位内
        bool within_limits = true;
        for (size_t j = 0; j < solution.size(); ++j)
        {
            if (solution[j] < joint_limits[j].first || solution[j] > joint_limits[j].second)
            {
                within_limits = false;
                break;
            }
        }

        if (within_limits)
        {
            // ✅ 对周期关节做归一化处理
            std::vector<size_t> normalize_indices = {3, 5};
            for (size_t joint_index : normalize_indices)
            {
                if (joint_index < solution.size() && joint_index < initial_q.size())
                {
                    solution[joint_index] = normalizeAngleToNearest(solution[joint_index], initial_q[joint_index]);
                }
            }

            all_solutions.push_back(solution);

            float distance = calculateDistance(initial_q, solution);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_solution = solution;
            }
        }
    }

    return {all_solutions, closest_solution};
}