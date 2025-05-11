import os
import rospkg
import pandas as pd
import matplotlib.pyplot as plt

# === 获取 ROS 包路径 ===
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('robot_control')
csv_path = os.path.join(pkg_path, 'data/admittance_data.csv')
plot_dir = os.path.join(pkg_path, 'plots')
os.makedirs(plot_dir, exist_ok=True)

# === 加载数据 ===
df = pd.read_csv(csv_path)

# === 定义每个实验的时间段（单位秒）===
time_ranges = [
    (0, 10),    # Exp 1
    (10, 20),   # Exp 2
    (25, 35),   # Exp 3
    (42, 52),   # Exp 4
]

# === 遍历每段实验 ===
for i, (t_start, t_end) in enumerate(time_ranges):
    df_exp = df[(df["time"] >= t_start) & (df["time"] <= t_end)].copy()
    if df_exp.empty:
        print(f"[警告] 实验 {i+1} 没有匹配数据，跳过。")
        continue

    # 时间归零处理
    df_exp["time"] -= df_exp["time"].iloc[0]
    time = df_exp["time"]

    # === 绘制力/扭矩图 ===
    plt.figure(figsize=(10, 6))
    plt.plot(time, df_exp["Fx"], label="Fx (N)")
    plt.plot(time, df_exp["Fy"], label="Fy (N)")
    plt.plot(time, df_exp["Fz"], label="Fz (N)")
    plt.plot(time, df_exp["Tx"], label="Tx (Nm)", linestyle='--')
    plt.plot(time, df_exp["Ty"], label="Ty (Nm)", linestyle='--')
    plt.plot(time, df_exp["Tz"], label="Tz (Nm)", linestyle='--')
    plt.title(f"[Exp {i+1}] Force/Torque vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N) / Torque (Nm)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(plot_dir, f"force_torque_{i+1}.png"))
    plt.close()

    # === 绘制位移/姿态图（双 y 轴）===
    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax1.plot(time, df_exp["dx"], label="dx (m)", color='tab:blue')
    ax1.plot(time, df_exp["dy"], label="dy (m)", color='tab:green')
    ax1.plot(time, df_exp["dz"], label="dz (m)", color='tab:orange')
    ax1.set_ylabel("Displacement (m)")
    ax1.set_ylim(-0.025, 0.025)
    ax1.set_xlabel("Time (s)")
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.plot(time, df_exp["dRx"], label="dRx (°)", linestyle='--', color='tab:red')
    ax2.plot(time, df_exp["dRy"], label="dRy (°)", linestyle='--', color='tab:purple')
    ax2.plot(time, df_exp["dRz"], label="dRz (°)", linestyle='--', color='tab:brown')
    ax2.set_ylabel("Rotation (deg)")
    ax2.set_ylim(-10, 10)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    plt.title(f"[Exp {i+1}] End-Effector Response vs Time")
    plt.tight_layout()
    plt.savefig(os.path.join(plot_dir, f"position_rotation_{i+1}.png"))
    plt.close()
