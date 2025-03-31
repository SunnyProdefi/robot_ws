#ifndef GRIPPER_DRIVER_H
#define GRIPPER_DRIVER_H

#include "gripper_sdk/canusb.h"
#include "robot_control/motor_driver.h"
#include <iostream>
#include <vector>

extern double q_send[BRANCHN_N][MOTOR_BRANCHN_N];

extern double q_recv[BRANCHN_N][MOTOR_BRANCHN_N];

extern double q_init[BRANCHN_N][MOTOR_BRANCHN_N];

class GripperDriver
{
public:
    GripperDriver(const char *tty_device, CANUSB_SPEED speed, int baudrate, const char *send_id);

    ~GripperDriver();

    int Init();
    void gripper_control(double pos, double vel, double force, double acc, double dec);

    int getBranchID();  // 添加获取 branch ID 的方法

private:
    int tty_fd_;
    const char *tty_device_;
    CANUSB_SPEED speed_;
    int baudrate_;
    const char *send_id_;
    unsigned char recv_data_[32];

    void generateCANMessage(double pos, double vel, double force, double acc, double dec, unsigned char *can_msg);
    void send_recv_frame(const unsigned char *send_data);
    void parseStatusFrame();
};

#endif  // GRIPPER_DRIVER_H
