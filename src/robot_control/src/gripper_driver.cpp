#include "gripper_driver.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <unordered_map>

GripperDriver::GripperDriver(const char *tty_device, CANUSB_SPEED speed, int baudrate, const char *send_id) : tty_device_(tty_device), speed_(speed), baudrate_(baudrate), send_id_(send_id) { tty_fd_ = -1; }

GripperDriver::~GripperDriver()
{
    if (tty_fd_ != -1)
    {
        close(tty_fd_);
    }
}

int GripperDriver::Init()
{
    tty_fd_ = adapter_init(tty_device_, baudrate_);
    if (tty_fd_ == -1)
    {
        std::cerr << "Failed to initialize TTY device" << std::endl;
        return -1;
    }

    if (command_settings(tty_fd_, speed_, CANUSB_MODE_NORMAL, CANUSB_FRAME_STANDARD) < 0)
    {
        std::cerr << "Failed to configure CAN settings" << std::endl;
        return -1;
    }

    return 0;
}

int GripperDriver::getBranchID()
{
    // 定义 CAN ID 与 curBranch 的映射关系
    std::unordered_map<std::string, int> id_to_branch = {{"01", 0}, {"02", 1}, {"03", 2}, {"04", 3}};

    // 查找 send_id_ 对应的 branch ID
    auto it = id_to_branch.find(send_id_);
    if (it != id_to_branch.end())
    {
        return it->second;
    }

    // 如果找不到，返回默认 branch ID（例如 0）
    return 0;
}

void GripperDriver::generateCANMessage(double pos, double vel, double force, double acc, double dec, unsigned char *can_msg)
{
    can_msg[0] = 0x00;                                       // Reserved
    can_msg[1] = static_cast<unsigned char>(pos * 255.0);    // Pos Cmd (0x00 - 0xFF)
    can_msg[2] = static_cast<unsigned char>(force * 255.0);  // Force Cmd
    can_msg[3] = static_cast<unsigned char>(vel * 255.0);    // Vel Cmd
    can_msg[4] = static_cast<unsigned char>(acc * 255.0);    // Acc Cmd
    can_msg[5] = static_cast<unsigned char>(dec * 255.0);    // Dec Cmd
    can_msg[6] = 0x00;                                       // Reserved
    can_msg[7] = 0x00;                                       // Reserved
}

void GripperDriver::send_recv_frame(const unsigned char *send_data)
{
    std::vector<char> send_data_str(32, 0);
    snprintf(send_data_str.data(), 32, "%02X %02X %02X %02X %02X %02X %02X %02X", send_data[0], send_data[1], send_data[2], send_data[3], send_data[4], send_data[5], send_data[6], send_data[7]);

    std::cout << "Sending CAN frame: " << send_data_str.data() << std::endl;

    if (inject_data_frame(tty_fd_, send_id_, send_data_str.data()) == -1)
    {
        std::cerr << "Error injecting data frame" << std::endl;
        return;
    }

    int frame_len = frame_recv(tty_fd_, recv_data_, sizeof(recv_data_));
    if (frame_len == -1)
    {
        std::cerr << "Error receiving frame" << std::endl;
        return;
    }

    if (frame_len > 0)
    {
        std::cout << "Received frame of length " << frame_len << ": ";
        for (int i = 0; i < frame_len; i++)
        {
            printf("%02X ", recv_data_[i]);
        }
        std::cout << std::endl;
    }
}

double GripperDriver::parseStatusFrame()
{
    if (recv_data_[0] == 0)
    {
        std::cerr << "Invalid frame" << std::endl;
        return -1.0;
    }

    unsigned char can_node_id = recv_data_[2];
    unsigned char fault_code = recv_data_[4];
    unsigned char state = recv_data_[5];
    unsigned char pos = recv_data_[6];
    unsigned char vel = recv_data_[7];
    unsigned char force = recv_data_[8];

    int curBranch = getBranchID();  // 通过 CAN ID 获取 curBranch
    int curBody = 6;                // 固定 Body 索引

    return static_cast<double>(pos) / 255.0;  // 归一化到 [0,1]
}

double GripperDriver::gripper_control(double pos, double vel, double force, double acc, double dec)
{
    unsigned char can_msg[8];
    generateCANMessage(pos, vel, force, acc, dec, can_msg);
    send_recv_frame(can_msg);
    return parseStatusFrame();
}
