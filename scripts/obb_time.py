from pathlib import Path
BACON_REPO_ROOT = Path(__file__).resolve().parents[1]

import os
import pandas as pd
from tabulate import tabulate

def count_lines(file_path):
    """
    统计文件中包含 'AABB' 和 'OBB' 的行数。

    参数:
    - file_path (str): 文件路径。

    返回:
    - dict: 包含 'AABB' 和 'OBB' 的行数。
    """
    counts = {'AABB': 0, 'OBB': 0}
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                if 'AABB' in line:
                    counts['AABB'] += 1
                if 'OBB' in line:
                    counts['OBB'] += 1
    except Exception as e:
        print(f"读取文件 {file_path} 时出错: {e}")
    return counts

def generate_tables(base_dir='data'):
    """
    遍历指定目录下的所有 .txt 文件，统计每个文件中包含 'AABB' 和 'OBB' 的行数，
    并生成两个表格分别展示这些统计数据。

    参数:
    - base_dir (str): 基础目录路径。默认为 'data'。

    返回:
    - None
    """
    # 初始化数据存储
    data_env = {}
    data_no = {}

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
            env_size = os.path.splitext(txt)[0]  # 去除扩展名，例如 '16.txt' -> '16'
            file_path = os.path.join(robot_dir, txt)
            counts = count_lines(file_path)

            # 初始化字典结构
            if robot not in data_env:
                data_env[robot] = {}
                data_no[robot] = {}

            data_env[robot][env_size] = counts['AABB']
            data_no[robot][env_size] = counts['OBB']

    # 创建 DataFrame
    df_env = pd.DataFrame(data_env).fillna(0).astype(int).T
    df_no = pd.DataFrame(data_no).fillna(0).astype(int).T

    # 排序环境大小
    env_order = ['8', '16', '32', '48']
    df_env = df_env.reindex(columns=env_order)
    df_no = df_no.reindex(columns=env_order)

    # 打印表格
    print("\n包含 'AABB' 的行数统计：")
    print(tabulate(df_env * 2, headers='keys', tablefmt='pretty'))

    print("\n包含 'OBB' 的行数统计：")
    print(tabulate(df_no * 47, headers='keys', tablefmt='pretty'))

    # 将结果保存为 CSV 文件
    df_env.to_csv('aabb_time.csv')
    df_no.to_csv('obb_time.csv')
    print("\n统计结果已保存为 'obb_time.csv' 和 'aabb_time.csv'。")

if __name__ == "__main__":
    # 如果需要，可以通过命令行参数传递基础目录
    import argparse

    parser = argparse.ArgumentParser(description="统计每个 .txt 文件中包含 'AABB' 和 'OBB' 的行数，并生成表格。")
    parser.add_argument(
        '-d', '--directory',
        type=str,
        default=f'{BACON_REPO_ROOT}/exp/obb_time',
        help="基础目录路径（默认为 'data'）。"
    )

    args = parser.parse_args()
    generate_tables(args.directory)