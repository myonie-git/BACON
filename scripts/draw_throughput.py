# import matplotlib.pyplot as plt
# import pandas as pd
# import numpy as np

# # 读取CSV文件
# csv_file = "formatted_execution_times.csv"
# data = pd.read_csv(csv_file)

# # 确保Model-Env列数据为字符串
# data["Model-Env"] = data.iloc[:, 0].astype(str)

# # 提取障碍物数量和机器人类型
# data["Obstacle_Size"] = data["Model-Env"].str.extract(r"-(\d+)$")[0].astype(int)
# data["Robot_Type"] = data["Model-Env"].str.extract(r"^(\w+)-")[0]

# # 按障碍物数量分组
# obstacle_sizes = sorted(data["Obstacle_Size"].unique())
# robot_types = sorted(data["Robot_Type"].unique())

# # 准备绘图数据
# grouped_data = {size: [] for size in obstacle_sizes}
# for size in obstacle_sizes:
#     group = data[data["Obstacle_Size"] == size]
#     for robot in robot_types:
#         row = group[group["Robot_Type"] == robot]
#         if not row.empty:
#             grouped_data[size].append({
#                 "CPU": row["CPU"].values[0] if not pd.isna(row["CPU"].values[0]) else 0,
#                 "GPU": row["GPU"].values[0] if not pd.isna(row["GPU"].values[0]) else 0,
#                 "TIMER": row["TIMER"].values[0] if not pd.isna(row["TIMER"].values[0]) else 0,
#             })
#         else:
#             grouped_data[size].append({"CPU": 0, "GPU": 0, "TIMER": 0})

# # 绘制柱状图
# bar_width = 0.25
# x_indices = np.arange(len(robot_types))

# fig, axes = plt.subplots(len(obstacle_sizes), 1, figsize=(15, len(obstacle_sizes) * 4), sharex=True)
# colors = ["#ff9999", "#66b3ff", "#99ff99"]  # 柔和的红、蓝、绿

# for idx, (size, ax) in enumerate(zip(obstacle_sizes, axes)):
#     cpu_values = [item["CPU"] for item in grouped_data[size]]
#     gpu_values = [item["GPU"] for item in grouped_data[size]]
#     timer_values = [item["TIMER"] for item in grouped_data[size]]

#     # 绘制CPU、GPU和TIMER的柱状图
#     ax.bar(x_indices - bar_width, cpu_values, bar_width, label="CPU", color=colors[0], edgecolor="black")
#     ax.bar(x_indices, gpu_values, bar_width, label="GPU", color=colors[1], edgecolor="black")
#     ax.bar(x_indices + bar_width, timer_values, bar_width, label="TIMER", color=colors[2], edgecolor="black")

#     # 添加标题和标签
#     ax.set_title(f"Obstacle Size: {size} Obstacles", fontsize=14, fontweight="bold", pad=15)
#     ax.set_ylabel("Normalized Throughput", fontsize=12)
#     ax.tick_params(axis="y", labelsize=10)
#     ax.grid(axis="y", linestyle="--", alpha=0.7)

# # 添加X轴标签、机器人类型
# axes[-1].set_xticks(x_indices)
# axes[-1].set_xticklabels(robot_types, rotation=45, ha="right", fontsize=10)

# # 添加图例和总标题
# # fig.suptitle("Normalized Throughput Across Robots and Obstacle Sizes", fontsize=16, fontweight="bold", y=0.94)
# fig.text(0.5, 0.04, "Robot Types", ha="center", fontsize=12)

# # 调整布局
# plt.tight_layout(rect=[0, 0.05, 1, 0.94])

# # 保存图像
# output_image = "normalized_throughput_improved.png"
# plt.savefig(output_image, dpi=300)
# print(f"Plot saved as {output_image}")

# # 可选：显示图像
# # plt.show()
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

import matplotlib
matplotlib.use('Agg')

csv_file = "throughput.csv"
data = pd.read_csv(csv_file)

data["Model-Env"] = data.iloc[:, 0].astype(str)

data["Obstacle_Size"] = data["Model-Env"].str.extract(r"-(\d+)$")[0].astype(int)
data["Robot_Type"] = data["Model-Env"].str.extract(r"^(\w+)-")[0]

obstacle_sizes = sorted(data["Obstacle_Size"].unique())
robot_types = sorted(data["Robot_Type"].unique())

font_options = {'family':'sans-serif'}
font_legend_options = {'family':'sans-serif', 'size':35}
font_text_options = {'family':'sans-serif', 'size':18}

# 准备绘图数据
grouped_data = {size: [] for size in obstacle_sizes}

for size in obstacle_sizes:
    group = data[data["Obstacle_Size"] == size]
    for robot in robot_types:
        row = group[group["Robot_Type"] == robot]
        if not row.empty:
            grouped_data[size].append({
                "Robot_Type": robot,
                "CPU": row["CPU"].values[0] if not pd.isna(row["CPU"].values[0]) else 0,
                "GPU": row["GPU"].values[0] if not pd.isna(row["GPU"].values[0]) else 0,
                "TIMER": row["TIMER"].values[0] if not pd.isna(row["TIMER"].values[0]) else 0,
            })
        else:
            grouped_data[size].append({
                "Robot_Type": robot,
                "CPU": 0,
                "GPU": 0,
                "TIMER": 0
            })

# 绘制柱状图
width = 0.2
plt.rcParams['hatch.linewidth'] = 2.0

n_cols = 4
n_rows = 1
fig, axes = plt.subplots(n_rows, n_cols, figsize=(18, 12), sharex=True)

total_subplots = n_rows * n_cols
if len(obstacle_sizes) < total_subplots:
    for i in range(len(obstacle_sizes), total_subplots):
        fig.delaxes(axes.flatten()[i])

hatches = ['//', '--', '\\\\']
edge_colors = ['#5CB0C3', '#8498AB', '#CCD376']
labels = ["CPU", "GPU", "TIMER"]

for idx, size in enumerate(obstacle_sizes):
    row = idx // n_cols
    col = idx % n_cols
    ax = axes[row, col] if n_rows > 1 else axes[col]
    
    data_list = grouped_data[size]
    x_indices = np.arange(len(data_list))
    cpu_values = [item["CPU"] for item in data_list]
    gpu_values = [item["GPU"] for item in data_list]
    timer_values = [item["TIMER"] for item in data_list]
    robot_labels = [item["Robot_Type"] for item in data_list]
    
    ax.bar(x_indices - width, cpu_values, width=width, label="CPU", hatch=hatches[0], edgecolor=edge_colors[0], facecolor='white', linewidth=2)
    ax.bar(x_indices, gpu_values, width=width, label="GPU", hatch=hatches[1], edgecolor=edge_colors[1], facecolor='white', linewidth=2)
    ax.bar(x_indices + width, timer_values, width=width, label="FPGA", hatch=hatches[2], edgecolor=edge_colors[2], facecolor='white', linewidth=2)
    
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    ax.set_ylabel('Normalized Throughput', size='35', fontdict=font_options)
    ax.set_title(f"{size} Obstacles", fontsize=35, fontweight="bold", pad=15)
    ax.tick_params(axis='x', labelsize=35)
    ax.tick_params(axis='y', labelsize=35)
    ax.set_xticks(x_indices)
    # ax.set_xticklabels(robot_labels, rotation=45, ha="right", fontdict=font_options)
    ax.set_xticklabels(robot_labels, fontdict=font_options)
    
    if idx == 0:
        ax.legend(loc='upper left', prop=font_legend_options, ncol=3)

fig.text(0.5, 0.01, "Robot Types", ha="center", fontsize=40, fontdict=font_options)

# plt.subplots_adjust(bottom=0.15, left=0.07, right=0.99, top=0.95, hspace=0.4, wspace=0.3)
plt.subplots_adjust(bottom=0.15, left=0.02, right=0.99, top=0.9, hspace=0.4, wspace=0.1)
fig.set_size_inches(50, 8)

output_image = "throughput.pdf"
fig.savefig(output_image, format='pdf')
# fig.savefig(output_image, dpi=300)
print(f"Plot saved as {output_image}")

# plt.subplots_adjust(bottom=0.18, left=0.07, right=0.99, top=0.99)
# #plt.subplots_adjust(bottom=0.25)  # 默认为0.1，这里增加到0.3，具体数值可根据需要调整
# fig.set_size_inches(6 * 1.9 + 8, 3.75 * 1.75 )
# #plt.tight_layout(pad = 20)
# fig.savefig('Topo.pdf', format='pdf')