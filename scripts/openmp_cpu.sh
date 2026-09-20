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
    "${BACON_REPO_ROOT}/build/openmp_sim_cpu_panda"
    "${BACON_REPO_ROOT}/build/openmp_sim_cpu_prbt"
    "${BACON_REPO_ROOT}/build/openmp_sim_cpu_fanuc"
    "${BACON_REPO_ROOT}/build/openmp_sim_cpu_jaco2"
    "${BACON_REPO_ROOT}/build/openmp_sim_cpu_jaco3"
    "${BACON_REPO_ROOT}/build/openmp_sim_cpu_go1"
)

# 定义要运行的目录列表
directories=(
    "${BACON_REPO_ROOT}/env/48"
    "${BACON_REPO_ROOT}/env/32"
    "${BACON_REPO_ROOT}/env/16"
    "${BACON_REPO_ROOT}/env/8"
)



# 输出文件
output_file="openmp.csv"
error_log="openmp.log"

# 创建临时存储结果的关联数组
declare -A results

# 初始化输出文件，添加表头：Test_Script,Directory,t1,t2,...,t104
header="Test_Script,Directory"
for t in {1..104}; do
    header="$header,t$t"
done
echo "$header" > "$output_file"

# 清空或创建错误日志文件
> "$error_log"

# 遍历每个测试脚本和目录
for script in "${test_scripts[@]}"; do
    # 检查测试脚本是否存在并可执行
    if [ ! -x "$script" ]; then
        echo "警告: 测试脚本 $script 不存在或不可执行。跳过。" | tee -a "$error_log"
        continue
    fi

    for dir in "${directories[@]}"; do
        # 初始化该组合的结果数组
        row="$script,$dir"
        
        for t in {1..104}; do
            echo "正在运行: $script -d $dir -t $t"

            # 执行测试脚本，并捕获标准输出
            timer_output=$("$script" -d "$dir" -t "$t" 2> temp_error.log)
            exit_code=$?

            # 去除输出中的换行和多余空白
            timer_value=$(echo "$timer_output" | tr -d '\n\r' | xargs)

            # 验证 timer_value 是否为有效数字
            if ! [[ "$timer_value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
                timer_value="Invalid_Timer"
                echo "警告: 脚本 $script 在目录 $dir, 参数 -t $t 执行成功，但无法解析 timer 值。实际输出为: '$timer_output'" | tee -a "$error_log"
            fi

            # 根据返回值添加结果
            if [ $exit_code -eq 0 ]; then
                row="$row,$timer_value"
            else
                row="$row,Error"
                echo "警告: 脚本 $script 在目录 $dir, 参数 -t $t 执行失败。详细错误请查看 $error_log"
                echo "脚本: $script, 目录: $dir, 参数 -t: $t" >> "$error_log"
                cat temp_error.log >> "$error_log"
                echo "-----------------------------" >> "$error_log"
            fi
        done
        
        # 将该组合的完整结果行写入文件
        echo "$row" >> "$output_file"
    done
done

# 移除临时错误输出文件
rm -f temp_error.log

echo "结果已保存到 $output_file"
echo "错误日志已生成到 $error_log"
