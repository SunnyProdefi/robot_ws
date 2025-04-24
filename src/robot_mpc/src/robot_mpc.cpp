#include "robot_mpc/mpc_controller.h"
#include <ros/ros.h>
#include <ros/package.h>

int main()
{
    std::string urdf_path = ros::package::getPath("robot_mpc") + "/models/single_arm_pino.urdf";
    double delta_t = 0.01;
    int horizon = 10;

    MpcController controller(urdf_path, delta_t, horizon);

    Eigen::VectorXd init_state(12);  // 假设 6 自由度
    init_state.setZero();

    controller.RunMpcSimulation(init_state, 50);

    return 0;
}
