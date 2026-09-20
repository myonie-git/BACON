from pathlib import Path
BACON_REPO_ROOT = Path(__file__).resolve().parents[1]

import subprocess
import os
import csv

# 比较调度策略的Latency

# 定义测试脚本列表
test_scripts = [
    f"{BACON_REPO_ROOT}/build/sim_group_timer_fanuc --g --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_prbt --g --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_panda --g --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_jaco2 --g --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_jaco3 --g --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_go1 --g --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_fanuc --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_prbt --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_panda --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_jaco2 --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_jaco3 --o",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_go1 --o",
]

# 定义要运行的目录列表
directories = [
    f"{BACON_REPO_ROOT}/env/8",
    f"{BACON_REPO_ROOT}/env/16",
    f"{BACON_REPO_ROOT}/env/32",
    f"{BACON_REPO_ROOT}/env/48",
]

# 输出文件
output_file = "software_speedup.csv"
error_log = "software_speedup.log"

# 清空错误日志文件
with open(error_log, mode='w') as log:
    log.write("")

# 初始化 CSV 数据存储结构
data = {}

# 参数到分类的动态映射
def map_category(args):
    if "--g" in args:
        return "Grouped"
    elif "--s" in args:
        return "Serial"
    elif "--p" in args:
        return "Parallelism"
    elif "--o" in args:
        return "Not Grouped"
    return "Unknown"

# 遍历每个测试脚本和目录
for script_entry in test_scripts:
    # 分离脚本路径和参数
    parts = script_entry.split()
    script_path = parts[0]
    args = parts[1:]

    # 检查测试脚本是否存在并可执行
    if not os.access(script_path, os.X_OK):
        with open(error_log, mode='a') as log:
            log.write(f"警告: 测试脚本 {script_path} 不存在或不可执行。跳过。\n")
        continue

    # 确定类别
    category = map_category(args)
    if category == "Unknown":
        with open(error_log, mode='a') as log:
            log.write(f"警告: 无法识别的参数组合 '{' '.join(args)}' 在脚本 {script_path} 中。跳过。\n")
        continue

    # 提取机器人类型
    try:
        # 假设机器人类型位于脚本名称的最后一部分，例如 sim_group_timer_fanuc
        robot_type = script_path.split("_")[-1]
    except IndexError:
        with open(error_log, mode='a') as log:
            log.write(f"警告: 无法解析机器人类型从脚本名称 {script_path} 中。跳过。\n")
        continue

    for directory in directories:
        # 提取环境大小
        env_size = directory.split("/")[-1]

        key = f"{robot_type}-{env_size}"  # 键的格式: robot-env_size

        # 初始化存储结构
        if key not in data:
            data[key] = {
                "Grouped": "N/A",
                "Not Grouped": "N/A",
                "Parallelism": "N/A",
                "Serial": "N/A",
            }

        print(f"正在运行: {script_path} {' '.join(args)} -d {directory}")

        try:
            # 使用 subprocess 运行脚本，并捕获标准输出和错误
            command = [script_path] + args + ["-d", directory]
            process = subprocess.run(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            exit_code = process.returncode
            timer_output = process.stdout.strip()

            # 检查命令是否成功执行
            if exit_code == 0:
                # 验证 timer_output 是否为有效数字
                try:
                    timer_value = float(timer_output)
                except ValueError:
                    timer_value = "Invalid_Timer"
                    with open(error_log, mode='a') as log:
                        log.write(
                            f"警告: 脚本 {script_path} 在目录 {directory} 中执行成功，但无法解析 timer 值。\n"
                        )
            else:
                timer_value = "Error"
                with open(error_log, mode='a') as log:
                    log.write(
                        f"警告: 脚本 {script_path} 在目录 {directory} 中执行失败。\n"
                    )
                    log.write(f"错误信息:\n{process.stderr}\n")
                    log.write("-----------------------------\n")

            # 填充数据
            data[key][category] = timer_value

        except Exception as e:
            with open(error_log, mode='a') as log:
                log.write(
                    f"脚本 {script_path} 在目录 {directory} 中运行时发生错误: {str(e)}\n"
                )
                log.write("-----------------------------\n")

# 将结果写入 CSV 文件
with open(output_file, mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile)

    # 写入表头
    writer.writerow(["Robot Model-Env Size", "Grouped", "Not Grouped", "Parallelism", "Serial"])

    # 按所需格式写入每一行数据
    for env_size in ["8", "16", "32", "48"]:
        for robot_type in ["fanuc", "prbt", "panda", "jaco2", "jaco3", "go1"]:
            key = f"{robot_type}-{env_size}"
            if key in data:
                row = [
                    data[key].get("Grouped", "N/A"),
                    data[key].get("Not Grouped", "N/A"),
                    data[key].get("Parallelism", "N/A"),
                    data[key].get("Serial", "N/A"),
                ]
            else:
                row = ["N/A", "N/A", "N/A", "N/A"]
            writer.writerow([key] + row)

print(f"结果已保存到 {output_file}")
print(f"错误日志已生成到 {error_log}")
