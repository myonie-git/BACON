import sys

import matplotlib
# don't use xwindow
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import re
import sys

# 柱形图
# 1. 生成数据
np.random.seed(20230811)
font_options = {'family':'sans-serif'
    # 'style':'italic'
    # 'weight':'normal',
    #   'color':'red',
    #   'size':16
}
font_legend_options = {'family':'sans-serif',
    # 'style':'italic'
    # 'weight':'normal',
    #   'color':'red',
       'size':30
}
font_text_options = {'family':'sans-serif',
    # 'style':'italic'
    # 'weight':'normal',
    #   'color':'red',
       'size':28
}


mapping = [0.65, 0.45, 0.52, 0.86, 0.82, 0.67, 0.89, 0.97, 0.89, 0.93, 0.99, 0.99] 


# 2. 创建画布
fig = plt.figure(dpi=100)
ax = fig.add_subplot(111)

# 3. 绘制柱形图

width=0.7
plt.rcParams['hatch.linewidth'] = 2.0
# plt.bar([i for i in range(len(fps))], fps, width=width, color=(0.7, 0.0, 0.0))
plt.bar([i - 0 - (0 * width) for i in range(len(mapping))], mapping, width=width, label='Naive/Ours mapping', hatch = '/' * 2, edgecolor = '#5CB0C3', facecolor = 'white', linewidth=2)
#plt.bar([i - 0.005 - (0.5 * width) for i in range(len(mapping))], mapping, width=width, label='naive/best mapp', hatch = '-' * 2, edgecolor = '#8498AB', facecolor = 'white', linewidth=2)
#plt.bar([i - 0 - (0 * width) for i in range(len(mapping))], mapping, width=width, label='NPU_side(UVM)', hatch = '/\\' * 2, edgecolor = '#a28cc2', facecolor = 'white', linewidth=2)
#plt.bar([i + 0.005 + (0.5 * width) for i in range(len(MIG_c))], MIG_c, width=width, label='Ours utilization', hatch = '\\' * 2 , edgecolor = '#CCD376', facecolor = 'white', linewidth=2)
#plt.bar([i + 0.015 + (1.5 * width) for i in range(len(vRouter_c))], vRouter_c, width=width, label='Ours performance', hatch = '|' * 2, edgecolor = '#F5BE8F', facecolor = 'white', linewidth=2)
# plt.bar([i + 0.03 + (3 * width) for i in range(len(fps_Bert))], fps_Bert, width=width, label='Bert', hatch = '\\' * 5, edgecolor = 'black', facecolor = 'white')

# plt.bar([i+width for i in range(len(translate))], translate, width=width, label='Translation reg request', color=(0.7, 0.0, 0.0))
plt.axvline(x=2 + 1.5, color='grey', linestyle='-.', linewidth = 2, dashes=(5, 10))
plt.axvline(x=6 + 1.5, color='grey', linestyle='-.', linewidth = 2, dashes=(5, 10))
# 4. 设置样式
ax.grid(axis='y', linestyle='--')
# ax.set_axisbelow(True)
# ax.spines[['right', 'top']].set_color('C7')
# plt.xticks([0.02 - (2 * width)] + [(x+1)+width/2 for x in range(4)], ['Ours', 'IOTLB-4', 'IOTLB-8', 'IOTLB-16', 'IOTLB-32'], fontdict=font_options)
plt.xticks([(x) for x in range(12)], ['35', '28', '24', '12', '35', '28', '24', '12', '35', '28', '24', '12'], fontdict=font_options)
# ax.set_xlabel("IO METHOD", size='35', fontdict=font_options)
ax.set_ylabel('Performance', size='35', fontdict=font_options)
#ax.grid(linestyle='-', linewidth=0.3)
ax.tick_params(axis='x', labelsize=30)  # 设置x轴刻度数字粗细为12
ax.tick_params(axis='y', labelsize=30)  # 设置y轴刻度数字粗细为10
# for x,y in zip(range(len(fps)),fps):
#     if (x == 0):
#         plt.text(x - 0.03,y + 0.04,y, fontdict=font_legend_options) #文本注解
#     elif (x == 4):
#         plt.text(x - 0.16,y + 0.04,y, fontdict=font_legend_options) #文本注解
#     else:
#         plt.text(x - 0.11,y + 0.04,y, fontdict=font_legend_options) #文本注解
# 添加“36 Core”和“48 Core”标签
ax.text(1 - 1.55, -0.3, "ResNet34 (usable/48 cores)", fontdict=font_text_options)
ax.text(5 - 1.25, -0.3, "ResNet18 (usable/48 cores)", fontdict=font_text_options)
ax.text(9 - 0.75, -0.3, "gpt2 (usable/48 cores)", fontdict=font_text_options)
ax.legend(loc='upper left', prop = font_legend_options, ncol=2)
plt.ylim(0, 1.9)
# plt.xlim(2786, 45000)
# plt.show()

# fig = plt.gcf()
plt.subplots_adjust(bottom=0.18, left=0.07, right=0.99, top=0.99)
#plt.subplots_adjust(bottom=0.25)  # 默认为0.1，这里增加到0.3，具体数值可根据需要调整
fig.set_size_inches(6 * 1.9 + 8, 3.75 * 1.75 )
#plt.tight_layout(pad = 20)
fig.savefig('Topo.pdf', format='pdf')