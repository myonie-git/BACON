from pathlib import Path
BACON_REPO_ROOT = Path(__file__).resolve().parents[1]

import os
import pandas as pd
from tabulate import tabulate
import re

def count_numbers(file_path):
    """
    统计文件中每个数字出现的次数。

    参数:
    - file_path (str): 文件路径。

    返回:
    - dict: 包含数字及其出现次数的字典。
    """
    counts = {}
    number_pattern = re.compile(r'\b\d+\b')  # 匹配独立的数字
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                numbers = number_pattern.findall(line)
                for num in numbers:
                    counts[num] = counts.get(num, 0) + 1
    except Exception as e:
        print(f"读取文件 {file_path} 时出错: {e}")
    return counts

def generate_tables(base_dir='data'):
    """
    遍历指定目录下的所有 .txt 文件，统计每个文件中每个数字出现的次数，
    并生成表格展示这些统计数据。

    参数:
    - base_dir (str): 基础目录路径。默认为 'data'。

    返回:
    - None
    """
    # 初始化数据存储
    data = []

    # 获取所有子目录（机器人模型）
    try:
        robot_models = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
    except FileNotFoundError:
        print(f"错误: 基础目录 '{base_dir}' 不存在。")
        return

    # 遍历每个机器人模型目录
    for robot in robot_models:
        robot_dir = os.path.join(base_dir, robot)
        try:
            txt_files = [f for f in os.listdir(robot_dir) if f.endswith('.txt')]
        except Exception as e:
            print(f"读取目录 '{robot_dir}' 时出错: {e}")
            continue

        for txt in txt_files:
            file_name = os.path.splitext(txt)[0]  # 去除扩展名，例如 'file1.txt' -> 'file1'
            file_path = os.path.join(robot_dir, txt)
            counts = count_numbers(file_path)

            # 构建行数据，包含机器人名称和文件名
            row_label = f"{robot}/{file_name}"
            row_data = {'File': row_label}
            row_data.update({num: int(count) for num, count in counts.items()})
            data.append(row_data)

    if not data:
        print("没有找到任何数据来生成表格。")
        return

    # 找出所有出现过的数字，以便作为表格的列
    all_numbers = set()
    for entry in data:
        all_numbers.update(entry.keys())
    all_numbers.discard('File')  # 移除 'File' 字段
    all_numbers = sorted(all_numbers, key=lambda x: int(x))  # 按数字大小排序

    # 确定DataFrame的列顺序
    columns = ['File'] + all_numbers

    # 创建 DataFrame
    df = pd.DataFrame(data, columns=columns).fillna(0)

    # 将所有数字列转换为整数类型
    for num in all_numbers:
        df[num] = df[num].astype(int)

    # 打印表格
    print("\n每个文件中每个数字出现的次数统计：")
    print(tabulate(df, headers='keys', tablefmt='pretty', showindex=False))

    # 将结果保存为 CSV 文件
    df.to_csv('number_counts.csv', index=False)
    print("\n统计结果已保存为 'number_counts.csv'。")

if __name__ == "__main__":
    # 如果需要，可以通过命令行参数传递基础目录
    import argparse

    parser = argparse.ArgumentParser(description="统计每个 .txt 文件中每个数字出现的次数，并生成表格。")
    parser.add_argument(
        '-d', '--directory',
        type=str,
        default=f'{BACON_REPO_ROOT}/exp/collision_id/',
        help=f"基础目录路径（默认为 '{BACON_REPO_ROOT}/exp/collision_id/'）。"
    )

    args = parser.parse_args()
    generate_tables(args.directory)
