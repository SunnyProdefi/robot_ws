import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_positions_from_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    positions = [entry['position'] for entry in data]
    return zip(*positions)  # x, y, z

def main():
    # 机身工作空间文件路径
    file_base = "/home/prodefi/github/robot_ws/src/robot_planning/config/feasible_base_poses.yaml"

    # 加载机身位置数据
    x, y, z = load_positions_from_yaml(file_base)

    # 创建 3D 图
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Workspace of Base")

    # 绘制机身工作空间
    ax.scatter(x, y, z, c='red', s=10, alpha=0.5, label='Base Workspace')

    # 轴标签
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
