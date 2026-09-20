import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

import matplotlib
matplotlib.use('Agg')  # Use 'Agg' backend for non-GUI environments

# 定义文件路径
env_counts_file = "obb_time.csv"
no_counts_file = "aabb_time.csv"

# 读取 CSV 文件
env_data = pd.read_csv(env_counts_file, index_col=0)
no_data = pd.read_csv(no_counts_file, index_col=0)

# 检查两个 DataFrame 的索引和列是否匹配
if not env_data.index.equals(no_data.index) or not env_data.columns.equals(no_data.columns):
    raise ValueError("env_counts.csv 和 no_counts.csv 的索引或列不匹配。请确保它们具有相同的机器人模型和环境尺寸。")

# 提取机器人类型和环境尺寸
robot_types = env_data.index.tolist()
env_sizes = env_data.columns.tolist()

# 定义字体选项
font_options = {'family': 'sans-serif'}
font_legend_options = {'family': 'sans-serif', 'size': 35}
font_text_options = {'family': 'sans-serif', 'size': 18}

# 准备绘图数据
grouped_data = {size: [] for size in env_sizes}

for size in env_sizes:
    for robot in robot_types:
        env_count = env_data.loc[robot, size] if not pd.isna(env_data.loc[robot, size]) else 0
        no_count = no_data.loc[robot, size] if not pd.isna(no_data.loc[robot, size]) else 0
        grouped_data[size].append({
            "Robot_Type": robot,
            "OBB_COUNT": env_count,
            "AABB_COUNT": no_count,
        })

# 绘图参数
width = 0.35  # 条形图的宽度
plt.rcParams['hatch.linewidth'] = 2.0

# 定义子图网格
n_cols = 2
n_rows = 2
fig, axes = plt.subplots(n_rows, n_cols, figsize=(50, 8), sharex=True, sharey=True)

total_subplots = n_rows * n_cols
if len(env_sizes) < total_subplots:
    for i in range(len(env_sizes), total_subplots):
        fig.delaxes(axes.flatten()[i])

# 定义图案和颜色
hatches = ['//', '\\\\']
edge_colors = ['#5CB0C3', '#8498AB']
labels = ["Env Counts", "No Counts"]

for idx, size in enumerate(env_sizes):
    row = idx // n_cols
    col = idx % n_cols
    ax = axes[row, col] if n_rows > 1 else axes[col]
    
    data_list = grouped_data[size]
    x_indices = np.arange(len(data_list))
    env_counts = [item["OBB_COUNT"] for item in data_list]
    no_counts = [item["AABB_COUNT"] for item in data_list]
    robot_labels = [item["Robot_Type"] for item in data_list]
    
    # 绘制 Env Counts 条形图
    ax.bar(x_indices - width/2, env_counts, width=width, label="Env Counts",
           hatch=hatches[0], edgecolor=edge_colors[0],
           facecolor='white', linewidth=2)
    
    # 绘制 No Counts 条形图
    ax.bar(x_indices + width/2, no_counts, width=width, label="No Counts",
           hatch=hatches[1], edgecolor=edge_colors[1],
           facecolor='white', linewidth=2)
    
    # 自定义坐标轴
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    ax.set_ylabel('Counts', size=35, fontdict=font_options)
    ax.set_title(f"{size} Obstacles", fontsize=35, fontweight="bold", pad=15)
    ax.tick_params(axis='x', labelsize=35)
    ax.tick_params(axis='y', labelsize=35)
    ax.set_xticks(x_indices)
    ax.set_xticklabels(robot_labels, fontdict=font_options)
    
    # 仅在第一个子图中添加图例
    if idx == 0:
        ax.legend(loc='upper left', prop=font_legend_options, ncol=2)

# 添加共用的 x 轴标签
fig.text(0.5, 0.01, "Robot Types", ha="center", fontsize=40, fontdict=font_options)

# 调整布局
plt.subplots_adjust(bottom=0.15, left=0.02, right=0.99, top=0.9, hspace=0.4, wspace=0.1)
fig.set_size_inches(25, 8)

# 保存图像
output_image = "obb_time.pdf"
fig.savefig(output_image, format='pdf')
print(f"Plot saved as {output_image}")

# import matplotlib.pyplot as plt
# import pandas as pd
# import numpy as np

# import matplotlib
# matplotlib.use('Agg')  # Use 'Agg' backend for non-GUI environments

# # Read the CSV files
# env_counts_file = "env_counts.csv"
# no_counts_file = "no_counts.csv"

# # Load data
# env_data = pd.read_csv(env_counts_file, index_col=0)
# no_data = pd.read_csv(no_counts_file, index_col=0)

# # Prepare plotting data
# sizes = env_data.index.tolist()  # Environment sizes
# robot_types = env_data.columns.tolist()  # Robot types

# # Plotting parameters
# width = 0.35  # Width of the bars
# plt.rcParams['hatch.linewidth'] = 2.0

# # Define subplot grid
# n_cols = 3
# n_rows = (len(sizes) + n_cols - 1) // n_cols  # Calculate number of rows needed
# fig, axes = plt.subplots(n_rows, n_cols, figsize=(18, 12), sharex=True)

# # Adjusting subplot layout if there are fewer sizes than total subplots
# total_subplots = n_rows * n_cols
# if len(sizes) < total_subplots:
#     for i in range(len(sizes), total_subplots):
#         fig.delaxes(axes.flatten()[i])

# # Define colors for the bars
# colors = ['#5CB0C3', '#8498AB']
# labels = ["Env Counts", "No Counts"]

# # Plotting the data
# for idx, size in enumerate(sizes):
#     row = idx // n_cols
#     col = idx % n_cols
#     ax = axes[row, col] if n_rows > 1 else axes[col]
    
#     env_values = env_data.loc[size].values
#     no_values = no_data.loc[size].values
    
#     x_indices = np.arange(len(robot_types))
    
#     # Plot Env Counts bars
#     ax.bar(x_indices - width/2, env_values, width=width, label="Env Counts",
#            color=colors[0], edgecolor='black', linewidth=2)
    
#     # Plot No Counts bars
#     ax.bar(x_indices + width/2, no_values, width=width, label="No Counts",
#            color=colors[1], edgecolor='black', linewidth=2)
    
#     # Customize the axes
#     ax.grid(axis='y', linestyle='--', alpha=0.7)
#     ax.set_ylabel('Counts', size=18)
#     ax.set_title(f"{size} Environment Size", fontsize=20, fontweight="bold", pad=15)
#     ax.tick_params(axis='x', labelsize=12)
#     ax.tick_params(axis='y', labelsize=12)
#     ax.set_xticks(x_indices)
#     ax.set_xticklabels(robot_types, rotation=45, ha='right')
    
#     # Add legend only to the first subplot
#     if idx == 0:
#         ax.legend(loc='upper left', fontsize=12)

# # Add a common x-label
# fig.text(0.5, 0.01, "Robot Types", ha="center", fontsize=20)

# # Adjust layout
# plt.subplots_adjust(bottom=0.15, left=0.05, right=0.95, top=0.9, hspace=0.4, wspace=0.1)
# fig.set_size_inches(25, 8)

# # Save the figure
# output_image = "collision_rate.pdf"
# fig.savefig(output_image, format='pdf')
# print(f"Plot saved as {output_image}")