roslaunch robot_control robot_control.launch

rostopic pub /control_flag std_msgs/Int32 "data: 1" -1

