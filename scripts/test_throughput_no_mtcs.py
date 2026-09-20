from pathlib import Path
BACON_REPO_ROOT = Path(__file__).resolve().parents[1]

import subprocess
import os
import csv
import shlex

# 定义测试脚本列表
test_scripts = [
    f"{BACON_REPO_ROOT}/build/sim_group_timer_fanuc --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_prbt  --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_panda --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_jaco2 --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_jaco3 --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_go1 --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_fanuc_no_mtcs --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_prbt_no_mtcs --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_panda_no_mtcs --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_jaco2_no_mtcs --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_jaco3_no_mtcs --g --t",
    f"{BACON_REPO_ROOT}/build/sim_group_timer_go1_no_mtcs --g --t",
]

# 定义要运行的目录列表
directories = [
    f"{BACON_REPO_ROOT}/env/8",
    f"{BACON_REPO_ROOT}/env/16",
    f"{BACON_REPO_ROOT}/env/32",
    f"{BACON_REPO_ROOT}/env/48",
]

# 输出文件
output_file = "throughput_no_mtcs.csv"
error_log = "throughput_no_mtcs.log"

# 清空错误日志文件
with open(error_log, mode='w') as log:
    log.write("")

# 初始化 CSV 数据存储结构
data = {}

# 遍历每个测试脚本和目录
for script in test_scripts:

    script_parts = shlex.split(script)
    script_path = script_parts[0]
    script_args = script_parts[1:]
    # 检查测试脚本是否存在并可执行
    if not os.access(script_path, os.X_OK):
        with open(error_log, mode='a') as log:
            log.write(f"警告: 测试脚本 {script} 不存在或不可执行。跳过。\n")
        continue

    for directory in directories:
        # 提取机器人类型和环境大小
        base_name = os.path.basename(script_path)
        base_name = base_name.replace("sim_group_timer_", "", 1)
        is_no_mtcs = base_name.endswith("_no_mtcs")
        robot_type = base_name[:-len("_no_mtcs")] if is_no_mtcs else base_name
        env_size = os.path.basename(directory)

        key = f"{robot_type}-{env_size}"  # 键的格式: robot-env_size

        # 初始化存储结构
        if key not in data:
            data[key] = {"CPU": "N/A", "GPU": "N/A", "TIMER": "N/A"}

        print(f"正在运行: {script} -d {directory}")

        try:
            # 使用 subprocess 运行脚本，并捕获标准输出和错误
            command = script_parts + ["-d", directory]
            process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
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
                        log.write(f"警告: 脚本 {script} 在目录 {directory} 中执行成功，但无法解析 timer 值。\n")
            else:
                timer_value = "Error"
                with open(error_log, mode='a') as log:
                    log.write(f"警告: 脚本 {script} 在目录 {directory} 中执行失败。\n")
                    log.write(f"错误信息:\n{process.stderr}\n")
                    log.write("-----------------------------\n")

            # 根据脚本分类填充数据
            if is_no_mtcs:
                data[key]["NO_MTCS"] = timer_value
            else:
                data[key]["TIMER"] = timer_value

        except Exception as e:
            with open(error_log, mode='a') as log:
                log.write(f"脚本 {script} 在目录 {directory} 中运行时发生错误: {str(e)}\n")
                log.write("-----------------------------\n")

# 将结果写入 CSV 文件
with open(output_file, mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile)

    # 写入表头
    writer.writerow(["robot model", "NO_MTCS", "TIMER"])

    # 按所需格式写入每一行数据
    for env_size in ["8", "16", "32", "48"]:
        for robot_type in ["fanuc", "prbt", "panda", "jaco2", "jaco3", "go1"]:
            key = f"{robot_type}-{env_size}"
            if key in data:
                row = [
                    data[key].get("NO_MTCS", "N/A"),
                    data[key].get("TIMER", "N/A"),
                ]
            else:
                row = ["N/A", "N/A"]
            writer.writerow([key] + row)

print(f"结果已保存到 {output_file}")
print(f"错误日志已生成到 {error_log}")
