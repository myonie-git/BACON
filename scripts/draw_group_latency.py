import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import matplotlib

# 使用 'Agg' 后端以在无图形界面环境下运行
matplotlib.use('Agg')

# 读取CSV文件
csv_file = "group_latency.csv"
data = pd.read_csv(csv_file)

# 确保第一列为字符串类型
data["Model-Env"] = data["Robot Model-Env Size"].astype(str)

# 提取障碍物数量和机器人类型
data["Obstacle_Size"] = data["Model-Env"].str.extract(r"-(\d+)$")[0].astype(int)
data["Robot_Type"] = data["Model-Env"].str.extract(r"^(\w+)-")[0]

# 获取唯一的障碍物数量和机器人类型
obstacle_sizes = sorted(data["Obstacle_Size"].unique())
robot_types = sorted(data["Robot_Type"].unique())

# 定义字体选项
font_options = {'family': 'sans-serif'}
font_legend_options = {'family': 'sans-serif', 'size': 20}
font_text_options = {'family': 'sans-serif', 'size': 15}

# 准备绘图数据
grouped_data = {size: [] for size in obstacle_sizes}

for size in obstacle_sizes:
    group = data[data["Obstacle_Size"] == size]
    for robot in robot_types:
        row = group[group["Robot_Type"] == robot]
        if not row.empty:
            grouped_data[size].append({
                "Robot_Type": robot,
                "Grouped": row["Grouped"].values[0] if not pd.isna(row["Grouped"].values[0]) else 0,
                "Not Grouped": row["Not Grouped"].values[0] if not pd.isna(row["Not Grouped"].values[0]) else 0,
                "Parallelism": row["Parallelism"].values[0] if not pd.isna(row["Parallelism"].values[0]) else 0,
                "Serial": row["Serial"].values[0] if not pd.isna(row["Serial"].values[0]) else 0,
            })
        else:
            grouped_data[size].append({
                "Robot_Type": robot,
                "Grouped": 0,
                "Not Grouped": 0,
                "Parallelism": 0,
                "Serial": 0
            })

# 设置绘图参数
width = 0.2  # 每个类别的柱状宽度
plt.rcParams['hatch.linewidth'] = 2.0  # 设置填充图案线宽

# 定义子图的行数和列数
n_cols = 4
n_rows = 1
fig, axes = plt.subplots(n_rows, n_cols, figsize=(24, 18), sharex=True, sharey=True)

# 确保子图数量与障碍物数量匹配
total_subplots = n_rows * n_cols
if len(obstacle_sizes) < total_subplots:
    for i in range(len(obstacle_sizes), total_subplots):
        fig.delaxes(axes.flatten()[i])

# 定义填充图案和边缘颜色
hatches = ['//', '--', '\\\\', 'xx']
edge_colors = ['#5CB0C3', '#8498AB', '#a28cc2', '#F5BE8F']
labels = ["Grouped", "Not Grouped", "Parallelism", "Serial"]

# 遍历每个障碍物数量并绘制对应的柱状图
for idx, size in enumerate(obstacle_sizes):
    row = idx // n_cols
    col = idx % n_cols
    ax = axes[row, col] if n_rows > 1 else axes[col]
    
    data_list = grouped_data[size]
    x_indices = np.arange(len(data_list))
    robot_labels = [item["Robot_Type"] for item in data_list]
    
    # 提取每个类别的值
    grouped_values = [item["Grouped"] for item in data_list]
    not_grouped_values = [item["Not Grouped"] for item in data_list]
    parallelism_values = [item["Parallelism"] for item in data_list]
    serial_values = [item["Serial"] for item in data_list]
    
    # 绘制四个类别的柱状图
    ax.bar(x_indices - 1.5*width, grouped_values, width=width, label="Grouped", hatch=hatches[0], edgecolor=edge_colors[0], facecolor='white', linewidth=2)
    ax.bar(x_indices - 0.5*width, not_grouped_values, width=width, label="Not Grouped", hatch=hatches[1], edgecolor=edge_colors[1], facecolor='white', linewidth=2)
    ax.bar(x_indices + 0.5*width, parallelism_values, width=width, label="Parallelism", hatch=hatches[2], edgecolor=edge_colors[2], facecolor='white', linewidth=2)
    ax.bar(x_indices + 1.5*width, serial_values, width=width, label="Serial", hatch=hatches[3], edgecolor=edge_colors[3], facecolor='white', linewidth=2)
    
    # 设置标题和标签
    ax.set_title(f"{size} Obstacles", fontsize=25, fontweight="bold", pad=15)
    ax.set_ylabel('Latency', size=20, fontdict=font_options)
    ax.tick_params(axis="y", labelsize=18)
    ax.tick_params(axis="x", labelsize=18)
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    
    # 设置X轴标签
    ax.set_xticks(x_indices)
    ax.set_xticklabels(robot_labels, rotation=45, ha="right", fontdict=font_options)
    
    # 仅在第一个子图中添加图例
    if idx == 0:
        ax.legend(loc='upper left', prop=font_legend_options, ncol=2)

# 添加全局X轴标签
fig.text(0.5, 0.04, "Robot Types", ha="center", fontsize=25, fontdict=font_options)
fig.text(0.04, 0.5, "Latency", va='center', rotation='vertical', fontsize=25, fontdict=font_options)
fig.set_size_inches(50, 8)

# 调整子图布局
plt.subplots_adjust(bottom=0.15, left=0.07, right=0.95, top=0.95, hspace=0.3, wspace=0.2)

# 保存图像为PDF
output_image = "strategy_latency.pdf"
fig.savefig(output_image, format='pdf')
print(f"Plot saved as {output_image}")
