import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager

# 自动使用系统中的中文字体（如 SimHei、Noto Sans CJK 等）
zh_font = None
for font in font_manager.findSystemFonts(fontpaths=None, fontext='ttf'):
    if any(name in font for name in ['SimHei', 'NotoSansCJK', 'Microsoft YaHei', 'PingFang']):
        zh_font = font
        break

if zh_font:
    matplotlib.rcParams['font.family'] = font_manager.FontProperties(fname=zh_font).get_name()
else:
    print("⚠️ 未找到中文字体，部分中文可能无法显示！")

# 防止负号显示为方块
matplotlib.rcParams['axes.unicode_minus'] = False

import os

def main():
    yaml_path = "/home/prodefi/github/robot_ws/src/robot_mpc/config/mpc_result_log.yaml"  # 修改为你的实际路径

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    trajectory_log = data.get("trajectory_log", [])

    steps = []
    actual_q_all = [[] for _ in range(6)]
    reference_q_all = [[] for _ in range(6)]

    for entry in trajectory_log:
        step = entry.get("step")
        steps.append(step)

        q = entry.get("q", [])
        target_traj = entry.get("target_traj", [])

        for i in range(6):
            actual_q_all[i].append(q[i] if i < len(q) else None)
            if target_traj and isinstance(target_traj[0], list):
                reference_q_all[i].append(target_traj[0][i])
            else:
                reference_q_all[i].append(None)

    # 将 step 映射为时间（单位：秒）
    total_time = 17.5
    num_steps = len(steps)
    time_axis = [step / (num_steps - 1) * total_time for step in steps]

    # 输出目录
    output_dir = "./mpc_tracking_plots"
    os.makedirs(output_dir, exist_ok=True)

    for i in range(6):
        plt.figure()
        plt.plot(time_axis, reference_q_all[i], label=f"期望值 q{i+1}")
        plt.plot(time_axis, actual_q_all[i], label=f"实际值 q{i+1}")
        plt.xlabel("时间（s）")
        plt.ylabel(f"关节角度 q{i+1}（rad）")
        plt.title(f"关节 q{i+1} 跟踪曲线")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        save_path = os.path.join(output_dir, f"mpc_tracking_q{i+1}.png")
        plt.savefig(save_path)
        print(f"图像已保存为 {save_path}")

if __name__ == "__main__":
    main()
