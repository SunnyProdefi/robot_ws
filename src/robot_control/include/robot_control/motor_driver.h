#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "innfos_sdk/actuatorcontroller.h"  // 包含机器人控制SDK相关头文件
// 分支电机个数
#define MOTOR_BRANCHN_N 7
#define BRANCHN_N 4
// 减速比
#define MOTOR_REDUCTION_RATIO_1 101.0
#define MOTOR_REDUCTION_RATIO_2 36.0
// 电机模式
#define MOTORCOMMAND_POSITION 1

extern std::vector<ActuatorController::UnifiedID> connectedActuators;
extern std::vector<int> connectedActuatorsID;
extern bool isEnable;

extern std::vector<std::vector<double>> q_send;
extern std::vector<std::vector<double>> q_recv;
extern std::vector<std::vector<double>> q_init;

int Inital_Motor_Connect();

int Motor_SendRec_One(const uint8_t& id, const double& set_value, const string& ipAddress, int type);
int Motor_Rec_One(const uint8_t& id, const string& ipAddress);

void Motor_SendRec_Func_ALL(int type);
int Motor_SendRec_Func_OneBranch(int branchi, int type);

void Motor_Rec_Func_ALL();
int Motor_Rec_Func_OneBranch(int branchi);

void Motor_Set_Func_ALL(int type);
void Motor_Set_Func_OneBranch(int branchi, int type);

void Set_Motor_One_Branchi_Enable(int branchi, int model);
void Set_Motor_ALL_Enable(int model);

#endif