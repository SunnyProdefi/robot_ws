# 机器人控制系统

该项目使用 ROS（Robot Operating System）来控制机器人，具体步骤如下：

## 1. 连接至主控

首先，你需要通过 `ssh` 连接到机器人主控（NUC）：

```bash
ssh -X robot@192.168.0.118
```

确保你在 **同一局域网** 中，Wi-Fi 网络设置如下：

- **Wi-Fi 名称**: `A505-5G`
- **Wi-Fi 密码**: `82339297A505`

## 2. 启动机器人控制系统

连接至主控后，使用以下命令启动机器人控制系统：

```bash
roslaunch robot_control robot_control.launch
roslaunch moveit_config demo.launch
```

`robot_control.launch` 是一个 ROS 启动文件，用于启动机器人控制系统的各个组件和节点。确保你已经安装并配置好 ROS 环境。

## 3. 发布控制信号

可以使用以下命令向 ROS 主题 `/control_flag` 发布控制信号，以控制机器人动作：

```bash
rostopic pub /control_flag std_msgs/Int32 "data: 1" -1
rosrun robot_rrt collision_check_node
```

这里：

- `rostopic` 是 ROS 的命令行工具，用于发布和订阅 ROS 主题消息。
- `/control_flag` 是你要发布消息的主题名称。
- `std_msgs/Int32` 表示消息的类型为整数（`Int32`）。
- `"data: 1"` 是发布的消息内容，`1` 是整数值（表示控制信号）。
- `-1` 表示只发布一次消息。

## 4. 进一步的操作

通过上述步骤，你可以成功启动并控制机器人。接下来的操作可以根据机器人控制系统的需求进行定制。

---

### 注意事项

1. 请确保机器人主控设备与 PC 主机处于同一局域网内。
2. 四个夹爪的 USB 接口必须依照顺序 `4-1-2-3` 接入主控的 USB 接口。
3. 若无法连接至主控，请首先检查网络设置是否正确。
4. 请确认 ROS 环境已正确配置，且所有所需依赖均已安装完毕。

---

### Tips

设置 ROS bag 存储路径：
```bash
export ROSBAG_PATH=/home/robot/rosbags/
```

在 launch 文件中启动 rosbag 录制节点：
```xml
<node name="rosbag_robot" pkg="rosbag" type="record" output="screen"
      args="-a -O $(env ROSBAG_PATH)robot_log.bag" />
```

启动机器人模型显示：
```bash
roslaunch robot_description init_robot_display.launch
```

启动 TF 监听节点：
```bash
rosrun robot_common listen_tf

rosrun robot_common listen_tf_workspace
```

---

### 控制标志说明（control_flag）

- `0`：仅接收当前实际位姿
- `1`：从当前位姿运动至初始位姿
- `2`：从初始位姿运动至目标位姿
- `3`：执行抓取操作
- `4`：完成取出操作
- `5`：完成交接操作
- `6`：完成放置操作
- `9`：归位

---

### 夹爪控制命令（control_flag 特殊值）

- `101`：全部夹爪闭合  
  ```cpp
  gripper_command.data = {1.0, 1.0, 1.0, 1.0};
  ```
- `102`：夹爪 2 与夹爪 3 半闭合，其余张开  
  ```cpp
  gripper_command.data = {0.0, 0.5, 0.5, 0.0};
  ```

**TODO：**

1. 废弃原USB延长线，改为加长通信线缆  
2. 接入六维力传感器线缆  
3. 更换主控接口法兰（3D打印件  
4. 设计并安装夹爪与桁架连接卡扣（3D打印件）  
5. 制作夹爪固定专用工装（机加工件）  

> **进度计划**：  
> 下周完成所有材料准备，下下周安排一天完成组装与调试。

更改control_flag == 2的插值点是
leg_transform_cs.cpp中的:
int point_per_stage = 1500;
int point = 6000;
和robot_planning.cpp中:
std::vector<std::vector<double>> joint_angles(6000, std::vector<double>(24, 0.0));
std::vector<Eigen::Matrix4f> base_sequence = robot_planning::interpolateFloatingBase(init_floating_base_file, gold_floating_base_file, 1500);


robot_rrt
third_party
boost
eigen3
hppfcl:sudo apt-get install ros-noetic-hpp-fcl
       2024.6.10：hpp-fcl 作者已将最新版 2.4.4 更新到 Ros Noetic 中，直接用上面的命令安装即可
ompl:  git clone https://github.com/ompl/ompl.git
pinocchio:sudo apt install robotpkg-py38-pinocchio
urdfdom
yaml-cpp

Q:单臂抓取步骤3和4中出现奇异的位姿,动作不流畅
A:改变抓取位姿
Q:moveit中base_link不动
A:在moveit配置中设置虚拟关节
<virtual_joint name="dummy" type="floating" parent_frame="world" child_link="dummy_root"/>

// 设定固定的 Serial_Num -> CAN 设备映射
std::string can1_serial = "31F100027B7";  // force_torque_data_3,4
std::string can0_serial = "31F10002876";  // force_torque_data_1,2

roslaunch robot_control robot_control.launch

roslaunch moveit_config demo.launch
rosrun robot_rrt add_truss_link_node
rostopic pub /control_flag std_msgs/Int32 "data: 1" -1
rosrun robot_rrt ompl_rrt_connect_node

rosrun robot_rrt collision_check_node

roslaunch robot_description init_robot_display.launch
rosrun robot_common listen_tf
rosrun robot_common listen_tf_workspace

夹爪抓取距离：0.13-0.15m (ikfast解析解：0.1m,距离六维力传感器末端)

你可以将以下内容直接复制粘贴到 `.md` 文件中使用，已使用 Markdown 格式规范排版：

---

## 相机到 `base_link` 的转换矩阵

在 URDF 中定义的 `camera_link` 相对于 `base_link` 的转换为：

### 🔹 平移向量 $\mathbf{P}$

$$
\mathbf{P} =
\begin{bmatrix}
0.05 \\
0.054 \\
-0.217
\end{bmatrix}
$$

---

### 🔹 旋转矩阵 $\mathbf{R}$

$$
\mathbf{R} =
\begin{bmatrix}
0     & -1     & 0      \\
0.760 & 0      & -0.649 \\
0.649 & 0      & 0.760
\end{bmatrix}
$$

---

### 🔹 齐次变换矩阵 $\mathbf{T}_{\text{camera} \to \text{base}}$

$$
\mathbf{T} =
\begin{bmatrix}
0     & -1     & 0      & 0.05 \\
0.760 & 0      & -0.649 & 0.054 \\
0.649 & 0      & 0.760  & -0.217 \\
0     & 0      & 0      & 1
\end{bmatrix}
$$


roslaunch robot_control robot_control.launch

rostopic pub /control_flag std_msgs/Int32 "data: 1" -1

rosrun robot_control force_const
rosrun robot_control force_sine
rosrun robot_control torque_disturb
rosrun robot_control force_mixed

rosrun robot_control force_zero

roslaunch robot_control robot_control.launch

rostopic pub /control_flag std_msgs/Int32 "data: 1" -1

rosrun robot_control grasp_pose_static_tf_publisher