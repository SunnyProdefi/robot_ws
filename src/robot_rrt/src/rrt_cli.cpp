#include <ros/ros.h>
#include <std_msgs/String.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>
#include <iostream>

int sock = -1;

void callback(const std_msgs::String::ConstPtr& msg)
{
    std::string message = msg->data + "\n";
    send(sock, message.c_str(), message.length(), 0);
    ROS_INFO_STREAM("[Send to C++] " << msg->data);

    char buffer[1024] = {0};
    int valread = read(sock, buffer, sizeof(buffer));
    if (valread > 0)
    {
        buffer[valread] = '\0';
        ROS_INFO_STREAM("[Received from C++] " << buffer);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_socket_client_cpp");
    ros::NodeHandle nh;

    // 建立 socket 连接
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        ROS_ERROR("Socket creation failed");
        return 1;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(9000);

    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
    {
        ROS_ERROR("Invalid address or address not supported");
        return 1;
    }

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
    {
        ROS_ERROR("Connection to server failed");
        return 1;
    }

    ROS_INFO("Connected to C++ socket server");

    ros::Subscriber sub = nh.subscribe("/test_socket_msg", 10, callback);
    ros::spin();

    close(sock);
    return 0;
}
