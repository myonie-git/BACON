from pathlib import Path
BACON_REPO_ROOT = Path(__file__).resolve().parents[1]

import argparse
import os
import re
from pathlib import Path

import pandas as pd
from tabulate import tabulate


NOT_HAPPENED_STR = "Collision: Not Happened"
ENV_COLLISION_STR = "Collision: Env Collision"


def parse_robot_env_from_filename(filename: str):
    """从类似 'fanuc_env8.log' 的文件名解析 robot 和 env_size。"""
    m = re.match(r"^(?P<robot>.+)_env(?P<env_size>\d+)\.log$", filename)
    if not m:
        return None, None
    return m.group("robot"), int(m.group("env_size"))


def count_collision_lines(file_path: str):
    """统计一个 log 文件里两种 collision 行的行数。"""
    counts = {"not_happened": 0, "env_collision": 0}
    try:
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                if NOT_HAPPENED_STR in line:
                    counts["not_happened"] += 1
                if ENV_COLLISION_STR in line:
                    counts["env_collision"] += 1
    except Exception as e:
        print(f"读取文件 {file_path} 时出错: {e}")
    return counts


def generate_csv(log_dir: str, output_csv: str, pivot: bool = False, print_table: bool = True):
    """遍历 log_dir 下的 .log 文件，统计两类 collision 行数并导出 CSV。"""
    log_dir_path = Path(log_dir)
    if not log_dir_path.exists():
        raise FileNotFoundError(f"目录不存在: {log_dir}")

    log_files = sorted([p for p in log_dir_path.iterdir() if p.is_file() and p.suffix == ".log"])
    if not log_files:
        raise FileNotFoundError(f"目录下未找到 .log 文件: {log_dir}")

    rows = []
    for p in log_files:
        robot, env_size = parse_robot_env_from_filename(p.name)
        counts = count_collision_lines(str(p))
        rows.append(
            {
                "robot": robot if robot is not None else "",
                "env_size": env_size if env_size is not None else -1,
                "log_file": p.name,
                "not_happened": counts["not_happened"],
                "env_collision": counts["env_collision"],
                "total": counts["not_happened"] + counts["env_collision"],
            }
        )

    df = pd.DataFrame(rows)
    df = df.sort_values(by=["robot", "env_size", "log_file"], ascending=[True, True, True])

    # 输出主 CSV（长表）
    output_csv_path = Path(output_csv)
    output_csv_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_csv_path, index=False)

    if print_table:
        print("\n每个 log 的 collision 统计：")
        print(tabulate(df, headers="keys", tablefmt="pretty", showindex=False))
        print(f"\n已保存 CSV: {output_csv_path}")

    # 可选：输出 pivot 表，和参考脚本类似
    if pivot:
        # 只对能解析出 robot/env_size 的条目做 pivot
        df_valid = df[(df["robot"] != "") & (df["env_size"] >= 0)].copy()
        if df_valid.empty:
            print("\n提示: 没有可用于 pivot 的数据（文件名未匹配 *_env<NUM>.log 格式）。")
            return

        env_order = [8, 16, 32, 48]
        existing_envs = sorted(set(df_valid["env_size"].tolist()))
        # 如果目录里的 env_size 不止这 4 种，就按实际出现的排序
        if any(e not in env_order for e in existing_envs):
            env_order = existing_envs

        df_not = df_valid.pivot(index="robot", columns="env_size", values="not_happened").fillna(0).astype(int)
        df_env = df_valid.pivot(index="robot", columns="env_size", values="env_collision").fillna(0).astype(int)

        df_not = df_not.reindex(columns=env_order)
        df_env = df_env.reindex(columns=env_order)

        pivot_not_path = output_csv_path.with_name(output_csv_path.stem + "_not_happened_pivot.csv")
        pivot_env_path = output_csv_path.with_name(output_csv_path.stem + "_env_collision_pivot.csv")
        df_not.to_csv(pivot_not_path)
        df_env.to_csv(pivot_env_path)

        if print_table:
            print("\nPivot：Collision: Not Happened 行数（robot x env_size）：")
            print(tabulate(df_not, headers="keys", tablefmt="pretty"))
            print("\nPivot：Collision: Env Collision 行数（robot x env_size）：")
            print(tabulate(df_env, headers="keys", tablefmt="pretty"))

        print(f"\n已保存 pivot CSV: {pivot_not_path} 和 {pivot_env_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "统计 moveit_envdir 日志中包含 'Collision: Not Happened' 和 'Collision: Env Collision' 的行数，并导出 CSV。"
        )
    )
    parser.add_argument(
        "-d",
        "--directory",
        type=str,
        default=f"{BACON_REPO_ROOT}/logs/moveit_envdir/20251213_160235",
        help="log 目录路径（例如 logs/moveit_envdir/20251213_160235）。",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="输出 CSV 路径（默认输出到 directory/collision_counts.csv）。",
    )
    parser.add_argument(
        "--pivot",
        action="store_true",
        help="额外输出两个 pivot CSV（robot x env_size），类似参考脚本的表格形式。",
    )
    parser.add_argument(
        "--no-print",
        action="store_true",
        help="不打印表格到终端，只输出 CSV。",
    )

    args = parser.parse_args()

    out = args.output
    if out is None:
        out = str(Path(args.directory) / "collision_counts.csv")

    generate_csv(args.directory, out, pivot=args.pivot, print_table=(not args.no_print))
