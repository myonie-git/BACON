#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# 定义要运行的测试脚本列表
test_scripts=(
    # "${BACON_REPO_ROOT}/build/test_faunc_time"
    # "${BACON_REPO_ROOT}/build/test_faunc_gpu_time"
    # "${BACON_REPO_ROOT}/build/test_prbt_time"
    # "${BACON_REPO_ROOT}/build/test_prbt_gpu_time"
    # "${BACON_REPO_ROOT}/build/test_panda_time"
    # "${BACON_REPO_ROOT}/build/test_panda_gpu_time"
    # "${BACON_REPO_ROOT}/build/test_jaco2_time"
    # "${BACON_REPO_ROOT}/build/test_jaco2_gpu_time"
    # "${BACON_REPO_ROOT}/build/test_jaco3_time"
    # "${BACON_REPO_ROOT}/build/test_jaco3_gpu_time"
    # "${BACON_REPO_ROOT}/build/test_go1_time"
    # "${BACON_REPO_ROOT}/build/test_go1_gpu_time"
    "${BACON_REPO_ROOT}/build/sim_timer"
)

# 定义要运行的目录列表
directories=(
    "${BACON_REPO_ROOT}/env/48-bak"
    # "${BACON_REPO_ROOT}/env/32"
    # "${BACON_REPO_ROOT}/env/16"
    # "${BACON_REPO_ROOT}/env/8"
)

# 输出文件
output_file="all_execution_times.csv"
error_log="all_execution_errors.log"

# 初始化输出文件，添加表头（新增 Timer(s) 列）
echo "Test_Script,Directory,Real_Time(s),User_Time(s),System_Time(s),Timer(s)" > "$output_file"

# 清空或创建错误日志文件
> "$error_log"

# 检查是否存在 GNU time
if ! command -v /usr/bin/time &> /dev/null; then
    echo "错误: GNU time 未找到。请确保安装了 GNU time 并且路径为 /usr/bin/time。"
    exit 1
fi

# 遍历每个测试脚本和每个目录
for script in "${test_scripts[@]}"; do
    # 检查测试脚本是否存在并可执行
    if [ ! -x "$script" ]; then
        echo "警告: 测试脚本 $script 不存在或不可执行。跳过。" | tee -a "$error_log"
        continue
    fi

    for dir in "${directories[@]}"; do
        echo "正在运行: $script -d $dir"

        # 使用 GNU time 测量命令执行时间
        # 将脚本的 stdout 重定向到 temp_output.log，stderr 捕获到 timing 变量
        timing=$(/usr/bin/time -f "%e,%U,%S" "$script" -d "$dir" 2>&1 1> temp_output.log)
        exit_code=$?

        # 初始化 timer_value
        timer_value="N/A"

        # 检查当前脚本是否为 sim_timer
        if [[ "$(basename "$script")" == "sim_timer" ]]; then
            # 从 temp_output.log 中读取 timer 值，并去除换行符
            timer_value=$(tr -d '\n\r' < temp_output.log)

            # 验证 timer_value 是否为有效数字
            if ! [[ "$timer_value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
                timer_value="Invalid_Timer"
                echo "警告: 脚本 $script 在目录 $dir 中执行成功，但无法解析 timer 值。" | tee -a "$error_log"
            fi
        fi

        # 检查命令是否成功执行
        if [ $exit_code -eq 0 ]; then
            if [[ "$(basename "$script")" == "sim_timer" ]]; then
                echo "$script,$dir,$timing,$timer_value" >> "$output_file"
            else
                echo "$script,$dir,$timing,$timer_value" >> "$output_file"
            fi
        else
            if [[ "$(basename "$script")" == "sim_timer" ]]; then
                echo "$script,$dir,Error,Error,Error,$timer_value" >> "$output_file"
            else
                echo "$script,$dir,Error,Error,Error,N/A" >> "$output_file"
            fi
            echo "警告: 脚本 $script 在目录 $dir 中执行失败。详细错误请查看 $error_log"
            # 将错误信息追加到错误日志文件中
            echo "脚本: $script, 目录: $dir" >> "$error_log"
            cat temp_output.log >> "$error_log"
            echo "-----------------------------" >> "$error_log"
        fi
    done
done

# 移除临时输出文件
rm -f temp_output.log

echo "结果已保存到 $output_file"
echo "错误日志已生成到 $error_log"
