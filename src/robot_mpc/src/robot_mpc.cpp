#include "robot_mpc/mpc_controller.h"
#include <ros/ros.h>
#include <ros/package.h>

int main()
{
    std::string urdf_path = ros::package::getPath("robot_mpc") + "/models/single_arm_pino.urdf";
    double delta_t = 0.01;
    int horizon = 10;

    MpcController controller(urdf_path, delta_t, horizon);

    Eigen::VectorXd init_state(12);                      // 假设 6 自由度
    init_state.head(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;  // 初始位置
    init_state.tail(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // 初始速度

    controller.RunMpcSimulation(init_state, 5000);

    return 0;
}
