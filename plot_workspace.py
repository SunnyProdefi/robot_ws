import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_positions_from_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    positions = [entry['position'] for entry in data]
    return zip(*positions)  # x, y, z

def main():
    # 文件路径
    file_branch2 = "/home/prodefi/github/robot_ws/src/robot_planning/config/workspace_branch2.yaml"
    file_branch3 = "/home/prodefi/github/robot_ws/src/robot_planning/config/workspace_branch3.yaml"

    # 加载位置数据
    x2, y2, z2 = load_positions_from_yaml(file_branch2)
    x3, y3, z3 = load_positions_from_yaml(file_branch3)

    # 创建 3D 图
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Workspace of Branch 2 and 3")

    # 绘制两个分支
    ax.scatter(x2, y2, z2, c='blue', s=10, alpha=0.5, label='Branch 2')
    ax.scatter(x3, y3, z3, c='green', s=10, alpha=0.5, label='Branch 3')

    # 轴标签
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
