#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# ===========================
# run_programs_with_screen.sh
# ===========================
# 该脚本使用 screen 运行多个程序，每个程序运行 3 次，并将输出保存到日志文件中。
# ===========================

# 检查是否安装了 screen
if ! command -v screen &> /dev/null
then
    echo "错误: screen 未安装。请使用以下命令安装 screen："
    echo "Ubuntu/Debian 系统: sudo apt-get install screen"
    echo "CentOS/Fedora 系统: sudo yum install screen"
    exit 1
fi

# 定义程序的绝对路径和日志文件路径
# 请根据实际路径修改以下变量
declare -A programs=(
    ["mtcs_step_prbt"]="${BACON_REPO_ROOT}/build/mtcs_step_prbt"
    ["mtcs_step_panda"]="${BACON_REPO_ROOT}/build/mtcs_step_panda"
    ["mtcs_step_fanuc"]="${BACON_REPO_ROOT}/build/mtcs_step_fanuc"
    ["mtcs_step_go1"]="${BACON_REPO_ROOT}/build/mtcs_step_go1"
    ["mtcs_step_jaco2"]="${BACON_REPO_ROOT}/build/mtcs_step_jaco2"
    ["mtcs_step_jaco3"]="${BACON_REPO_ROOT}/build/mtcs_step_jaco3"
)

declare -A logs_dir=(
    ["mtcs_step_prbt"]="${BACON_REPO_ROOT}/logs/mtcs_step_prbt"
    ["mtcs_step_panda"]="${BACON_REPO_ROOT}/logs/mtcs_step_panda"
    ["mtcs_step_fanuc"]="${BACON_REPO_ROOT}/logs/mtcs_step_fanuc"
    ["mtcs_step_go1"]="${BACON_REPO_ROOT}/logs/mtcs_step_go1"
    ["mtcs_step_jaco2"]="${BACON_REPO_ROOT}/logs/mtcs_step_jaco2"
    ["mtcs_step_jaco3"]="${BACON_REPO_ROOT}/logs/mtcs_step_jaco3"
)

# 创建日志目录（如果不存在）
for program in "${!logs_dir[@]}"; do
    mkdir -p "${logs_dir[$program]}"
done

# 定义每个程序的运行次数
RUN_COUNT=3

# 遍历每个程序并启动对应的 screen 会话
for program in "${!programs[@]}"; do
    cmd="${programs[$program]}"
    log_base_dir="${logs_dir[$program]}"
    
    for ((i=1; i<=RUN_COUNT; i++)); do
        # 获取当前时间戳
        timestamp=$(date +"%Y%m%d_%H%M%S")
        
        # 使用时间戳和运行次数确保日志文件名唯一
        log_file="${log_base_dir}/${program}_run${i}_${timestamp}.log"
        
        # 为每个运行创建唯一的会话名称
        session_name="${program}_run${i}_${timestamp}"
        
        # 检查该会话是否已经存在
        if screen -list | grep -q "\.${session_name}"; then
            echo "Screen 会话 '$session_name' 已经存在。跳过启动 '$cmd'。"
        else
            # 启动一个新的 screen 会话并在其中运行命令，输出重定向到日志文件
            screen -dmS "$session_name" bash -c "$cmd > \"$log_file\" 2>&1"
            echo "已启动 screen 会话 '$session_name' 运行命令 '$cmd'，日志文件: '$log_file'。"
            
            # 可选：添加延迟，避免同时启动太多进程
            sleep 1
        fi
    done
done

echo "所有程序的三个实例已通过 screen 启动。"

# 可选：显示当前所有 screen 会话
echo -e "\n当前的 screen 会话列表："
screen -list
