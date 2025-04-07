#include <math.h>
#include "robot_control/motor_driver.h"

ActuatorController* pController = nullptr;
std::vector<ActuatorController::UnifiedID> connectedActuators;
std::vector<int> connectedActuatorsID;
bool isEnable = false;

// 电机ID匹配
char User_MotorIDMap[BRANCHN_N][MOTOR_BRANCHN_N] = {{1, 2, 3, 4, 5, 6, 7}, {8, 9, 10, 11, 12, 13, 14}, {15, 16, 17, 18, 19, 20, 21}, {22, 23, 24, 25, 26, 27, 28}};

// 电机控制方向
const int User_Motor_Sign[BRANCHN_N][MOTOR_BRANCHN_N] = {{1, 1, -1, -1, -1, -1, -1}, {1, 1, -1, -1, -1, -1, -1}, {1, 1, -1, -1, -1, -1, -1}, {1, 1, -1, -1, -1, -1, -1}};

// 电机控制偏置
const double User_Motor_Offset[BRANCHN_N][MOTOR_BRANCHN_N] = {{0, 0, 0, -9, 0, -9, 0}, {0, 0, 0, -9, 0, -9, 0}, {0, 0, 0, -9, 0, -9, 0}, {0, 0, 0, -9, 0, -9, 0}};

// 初始化为 0 的二维 vector
std::vector<std::vector<double>> q_send = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

std::vector<std::vector<double>> q_recv = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

// std::vector<std::vector<double>> q_init = {
//     {1.597743, 0.2950242, 2.156446, 3.101645, -0.4948243, -0.01648967, 1.0}, {2.041711, -0.616538, 2.032447, -1.33452, 1.159544, -2.898303, 1.0}, {-2.041711, -0.616538, 2.032447, 1.33452, 1.159544, -0.24329, 1.0}, {-1.597743, 0.295025, 2.156445, -0.04025241, 0.4948952, 0.01637039, 1.0}};

std::vector<std::vector<double>> q_init = {
    {1.597743, 0.2950242, 2.156446, 3.101645, -0.4948243, -0.01648967, 1.0}, {2.041711, -0.616538, 2.032447, -1.33452, 1.159544, -2.898303, 1.0}, {-2.041711, -0.616538, 2.032447, 1.33452, 1.159544, -0.24329, 1.0}, {-1.597743, 0.295025, 2.156445, -0.04025241, 0.4948952, 0.01637039, 1.0}};

// 分支ID Map函数
bool GetMotorBranchRange(int branchi, int& startIdx, int& endIdx)
{
    switch (branchi)
    {
    case 0:
        startIdx = 1;
        endIdx = 6;
        break;  // 左前腿
    case 1:
        startIdx = 8;
        endIdx = 13;
        break;  // 左后腿
    case 2:
        startIdx = 15;
        endIdx = 20;
        break;  // 右后腿
    case 3:
        startIdx = 22;
        endIdx = 27;
        break;  // 右前腿
    default:
        std::cerr << "Invalid branch index. Valid values are 0-3." << std::endl;
        return false;
    }
    return true;
}

// 电机初始化
int Inital_Motor_Connect()
{
    // 仅在控制器未初始化时进行初始化
    if (pController == nullptr)
    {
        pController = ActuatorController::initController();
        if (pController == nullptr)
        {
            std::cerr << "Failed to initialize ActuatorController." << std::endl;
            return true;
        }
    }
    // 定义一个错误变量并查找执行器
    Actuator::ErrorsDefine ec;
    connectedActuators = pController->lookupActuators(ec);
    // 检查是否找到执行器
    if (!connectedActuators.empty())
    {
        for (const auto& actuatorID : connectedActuators)
        {
            std::cout << "Actuator ID: " << actuatorID.actuatorID << ", IP Address: " << actuatorID.ipAddress.c_str() << std::endl;
            connectedActuatorsID.push_back(actuatorID.actuatorID);
        }
    }
    else
    {
        // 错误处理
        if (ec == 0x803)
        {
            std::cerr << "Connection error: Communication with ECB (ECU) failed." << std::endl;
        }
        else if (ec == 0x802)
        {
            std::cerr << "Connection error: Communication with actuator failed." << std::endl;
        }
        else
        {
            std::cerr << "Connection error: Unknown error code: " << std::hex << ec << std::endl;
        }
        return false;
    }
    return true;
}

// 电机使能
void Set_Motor_One_Branchi_Enable(int branchi, int model)
{
    if (pController == nullptr)
    {
        return;
    }
    int startIdx = 0;
    int endIdx = 0;
    if (!GetMotorBranchRange(branchi, startIdx, endIdx))
    {
        return;
    }
    if (!connectedActuators.empty())
    {
        for (const auto& actuatorID : connectedActuators)
        {
            if (actuatorID.actuatorID >= startIdx && actuatorID.actuatorID <= endIdx)
            {
                uint8_t id = actuatorID.actuatorID;
                std::string ipAddress = actuatorID.ipAddress;
                bool success = false;
                if (model == 1)
                {
                    success = pController->enableActuator(id, ipAddress);
                }
                else if (model == 0)
                {
                    success = pController->disableActuator(id, ipAddress);
                }
                if (success)
                {
                    std::cout << (model == 1 ? "Successfully enabled " : "Successfully disabled ") << "actuator with ID " << static_cast<int>(id) << " in branch " << branchi << "." << std::endl;
                }
                else
                {
                    std::cerr << "Error " << (model == 1 ? "enabling" : "disabling") << " actuator with ID " << static_cast<int>(id) << std::endl;
                }
            }
        }
    }
    else
    {
        std::cerr << "No actuators found or communication error "s << std::endl;
        return;
    }
    isEnable = (model == 1);
}

void Set_Motor_ALL_Enable(int model)
{
    for (int branchi = 0; branchi < BRANCHN_N; branchi++)
    {
        Set_Motor_One_Branchi_Enable(branchi, model);
    }
}

// 设置电机模式(目前只有位置模式)
void Motor_Set_Func_OneBranch(int branchi, int type)
{
    if (pController == nullptr)
    {
        return;
    }
    int startIdx = 0;
    int endIdx = 0;
    if (!GetMotorBranchRange(branchi, startIdx, endIdx))
    {
        return;
    }
    for (const auto& actuatorID : connectedActuators)
    {
        if (actuatorID.actuatorID >= startIdx && actuatorID.actuatorID <= endIdx)
        {
            uint8_t id = actuatorID.actuatorID;
            std::string ipAddress = actuatorID.ipAddress;
            if (id - startIdx < 0)
            {
                continue;
            }
            // Mode_Profile_Pos / Mode_Pos // Mode_Profile_Vel / Mode_Vel
            if (type == MOTORCOMMAND_POSITION)
            {
                pController->activateActuatorMode(id, Actuator::Mode_Pos);
                std::cout << "Set actuator " << static_cast<int>(id) << " to POSITION mode." << std::endl;
            }
            else
            {
                return;
            }
        }
    }
}

void Motor_Set_Func_ALL(int type)
{
    // temp code
    if (isEnable == false)
    {
        return;
    }
    for (int branchi = 0; branchi < BRANCHN_N; branchi++)
    {
        Motor_Set_Func_OneBranch(branchi, type);
    }
}

// 单个电机收发一次
int Motor_SendRec_One(const uint8_t& id, const double& set_value, const string& ipAddress, int type)  // 单个电机收发一次
{
    if (type == MOTORCOMMAND_POSITION)
    {
        pController->setPosition(id, set_value);
    }
    // 获取电机当前的实际位置
    const double actuatorPosition = pController->getPosition(id, true, ipAddress);

    // 查找电机对应的分支和体节
    int curBranch = -1;
    int curBody = -1;
    // Lambda 函数用于查找电机位置
    auto findMotorPosition = [&](int& branch, int& body) -> bool
    {
        for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
        {
            for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; ++bodyi)
            {
                if (User_MotorIDMap[branchi][bodyi] == id)
                {
                    branch = branchi;
                    body = bodyi;
                    return true;
                }
            }
        }
        return false;
    };
    // 查找逻辑
    if (!findMotorPosition(curBranch, curBody))
    {
        std::cerr << "Failed to find branch and body for actuator ID: " << static_cast<int>(id) << std::endl;
        return false;
    }
    // 根据体节计算当前位置
    const double reductionRatio = (curBody <= 2) ? MOTOR_REDUCTION_RATIO_1 : MOTOR_REDUCTION_RATIO_2;
    const double curPosition = User_Motor_Sign[curBranch][curBody] * ((actuatorPosition - User_Motor_Offset[curBranch][curBody]) / reductionRatio) * M_PI * 2;

    // 传入q_recv
    q_recv[curBranch][curBody] = curPosition;
    cout << "Motor ID: " << static_cast<int>(id) << ", Position: " << curPosition << endl;
    return true;
}

// 单个电机只收一次
int Motor_Rec_One(const uint8_t& id, const string& ipAddress)
{
    // 获取电机当前位置
    const double actuatorPosition = pController->getPosition(id, true, ipAddress);
    // 查找电机对应的分支和体节
    int curBranch = -1;
    int curBody = -1;
    // 提取查找逻辑到函数中，避免代码重复
    auto findMotorPosition = [&]() -> bool
    {
        for (int branchi = 0; branchi < BRANCHN_N; ++branchi)
        {
            for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; ++bodyi)
            {
                if (User_MotorIDMap[branchi][bodyi] == id)
                {
                    curBranch = branchi;
                    curBody = bodyi;
                    return true;
                }
            }
        }
        return false;
    };
    if (!findMotorPosition())
    {
        std::cerr << "Failed to find branch and body for actuator ID: " << static_cast<int>(id) << std::endl;
        return false;
    }
    // 计算当前位置，根据体节决定减速比
    const double reductionRatio = (curBody <= 2) ? MOTOR_REDUCTION_RATIO_1 : MOTOR_REDUCTION_RATIO_2;
    const double curPosition = User_Motor_Sign[curBranch][curBody] * ((actuatorPosition - User_Motor_Offset[curBranch][curBody]) / reductionRatio) * M_PI * 2;

    //  根据体节正负号调整角度
    // cout << "Motor ID: " << static_cast<int>(id) << ", Position: " << curPosition << endl;
    q_recv[curBranch][curBody] = curPosition;
    return true;
}

// 电机控制-收发
int Motor_SendRec_Func_OneBranch(int branchi, int type)
{
    // 检查控制器是否初始化
    if (pController == nullptr)
    {
        return false;
    }
    // 获取分支范围
    int startIdx = -1, endIdx = -1;
    if (!GetMotorBranchRange(branchi, startIdx, endIdx))
    {
        return false;
    }
    // 检查是否启用
    if (!isEnable)
    {
        return true;
    }
    // 遍历连接的执行器
    if (!connectedActuators.empty())
    {
        for (const auto& actuator : connectedActuators)
        {
            uint8_t id = actuator.actuatorID;
            const std::string& ipAddress = actuator.ipAddress;
            // 检查是否在当前分支范围内
            if (id < startIdx || id > endIdx)
            {
                continue;
            }
            // 计算索引
            int localIdx = id - startIdx;
            if (localIdx < 0 || localIdx >= MOTOR_BRANCHN_N)
            {
                continue;
            }
            // 计算位置
            double reductionRatio = (localIdx <= 2) ? MOTOR_REDUCTION_RATIO_1 : MOTOR_REDUCTION_RATIO_2;
            double position = User_Motor_Sign[branchi][localIdx] * (q_send[branchi][localIdx] / (2 * M_PI)) * reductionRatio + User_Motor_Offset[branchi][localIdx];
            double set_value = 0;
            if (type == MOTORCOMMAND_POSITION)
            {
                set_value = position;
            }
            // 调用发送和接收方法
            cout << "Motor_SendRec_Func_OneBranch branchi: " << branchi << ", id: " << static_cast<int>(id) << ", set_value: " << set_value << endl;
            Motor_SendRec_One(id, set_value, ipAddress, type);
        }
    }
    return true;
}

void Motor_SendRec_Func_ALL(int type)
{
    // temp code
    if (isEnable == false)
    {
        return;
    }
    for (int branchi = 0; branchi < BRANCHN_N; branchi++)
    {
        cout << "Motor_SendRec_Func_ALL branchi: " << branchi << endl;
        Motor_SendRec_Func_OneBranch(branchi, type);
    }
}

// 电机控制-只收
int Motor_Rec_Func_OneBranch(int branchi)
{
    // 检查控制器是否初始化
    if (pController == nullptr)
    {
        return false;
    }
    // 获取分支范围
    int startIdx = 0, endIdx = 0;
    if (!GetMotorBranchRange(branchi, startIdx, endIdx))
    {
        return false;
    }
    // 检查是否启用
    if (!isEnable)
    {
        return true;
    }
    // 遍历连接的执行器
    for (const auto& actuator : connectedActuators)
    {
        uint8_t id = actuator.actuatorID;
        const std::string& ipAddress = actuator.ipAddress;
        // 检查是否在当前分支范围内
        if (id < startIdx || id > endIdx)
        {
            continue;
        }
        // 计算本地索引并验证范围
        int localIdx = id - startIdx;
        if (localIdx < 0 || localIdx >= MOTOR_BRANCHN_N)
        {
            continue;
        }
        // 调用接收方法
        Motor_Rec_One(id, ipAddress);
    }
    return true;
}

void Motor_Rec_Func_ALL()
{
    // temp code
    if (isEnable == false)
    {
        // std::cout << "Motor is not enabled." << std::endl;
        return;
    }
    for (int branchi = 0; branchi < BRANCHN_N; branchi++)
    {
        Motor_Rec_Func_OneBranch(branchi);
    }
}
