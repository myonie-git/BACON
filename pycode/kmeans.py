from pathlib import Path
BACON_REPO_ROOT = Path(__file__).resolve().parents[1]

import pandas as pd
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

# 加载CSV文件
filepath = f"{BACON_REPO_ROOT}/exp/exp4/prbt/collision_results.csv"
df = pd.read_csv(filepath, header=None)
df = df.drop(0)
df = df.transpose()

# 检查数据
print(df.head())

X = df.values

# 使用KMeans进行聚类
kmeans = KMeans(n_clusters=7, random_state=42)  # 设定聚类数目为5
kmeans.fit(X)

# 获取聚类结果
labels = kmeans.labels_
centroids = kmeans.cluster_centers_

# 将聚类结果添加到原始数据表中
df['Cluster'] = labels

# 输出每个样本的聚类结果
for i, label in enumerate(labels):
    print(f"样本 {i} 被归为第 {label} 类")

# 可视化聚类结果（假设数据是2维的）
# plt.scatter(X[:, 0], X[:, 1], c=labels, cmap='viridis')
# plt.scatter(centroids[:, 0], centroids[:, 1], c='red', marker='X', s=200, alpha=0.75)
# plt.xlabel('Feature 1')
# plt.ylabel('Feature 2')
# plt.title('K-Means Clustering')
# plt.show()

# 保存带有聚类结果的数据到新的CSV文件
df.to_csv('clustered_data.csv', index=False)
