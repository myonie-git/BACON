#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# 统一 seed 下，批量跑 sim_rrt_sample_limit 的六个机械臂与多组 time（使用 --time/--times 控制终止）。

set -euo pipefail

REPO="${REPO:-${BACON_REPO_ROOT}}"
BIN="${BIN:-${REPO}/build/sim_rrt_sample_limit}"
ENV_DIR="${ENV_DIR:-${REPO}/env/32}"
ENV_ID="${ENV_ID:-2}"
THREADS="${THREADS:-4}"   # 兼容参数：RRT 为单线程，程序内部会忽略该值
SEED="${SEED:-123456}"
OUT_CSV="${OUT_CSV:-${REPO}/scripts/rrt_time_limit_results.csv}"

if [[ ! -x "${BIN}" ]]; then
  echo "可执行文件不存在或不可执行: ${BIN}" >&2
  echo "请先在工程根目录编译，或通过 BIN 环境变量指定路径。" >&2
  exit 1
fi

robots=(fanuc go1 jaco2 jaco3 panda prbt)
# times=(0.01 0.02 0.05 0.1 0.2 0.5 1.0)

times=(0.0075 0.015 0.0375 0.075 0.15 0.375 0.75 1.5)

mkdir -p "$(dirname "${OUT_CSV}")"
printf "robot,time_s,total_ms,valid_wall_ms,wall_pct,valid_calls,actual_s,sampled\n" | tee "${OUT_CSV}"

for robot in "${robots[@]}"; do
  for t in "${times[@]}"; do
    log_file="$(mktemp)"
    if ! "${BIN}" \
      --robot "${robot}" \
      --threads "${THREADS}" \
      --time "${t}" \
      --seed "${SEED}" \
      --env-dir "${ENV_DIR}" \
      --env-id "${ENV_ID}" \
      --repo "${REPO}" \
      >"${log_file}" 2>&1; then
      echo "运行失败: robot=${robot}, time=${t}" >&2
      cat "${log_file}" >&2
      rm -f "${log_file}"
      exit 1
    fi

    total_ms="$(sed -n 's/.*总耗时 (wall):[[:space:]]*\([0-9.eE+-]\+\).*/\1/p' "${log_file}" | head -n1)"
    valid_ms="$(sed -n 's/.*状态有效性检查耗时(墙钟累计):[[:space:]]*\([0-9.eE+-]\+\).*/\1/p' "${log_file}" | head -n1)"
    wall_pct="$(sed -n 's/.*状态有效性检查耗时(墙钟累计):.*占墙钟约[[:space:]]*\([0-9.eE+-]\+\)%.*/\1/p' "${log_file}" | head -n1)"
    valid_calls="$(sed -n 's/.*状态有效性检查调用次数:[[:space:]]*\([0-9]\+\).*/\1/p' "${log_file}" | head -n1)"
    actual_s="$(sed -n 's|.*运行时间(目标/实际):[[:space:]]*[0-9.eE+-]\+[[:space:]]*/[[:space:]]*\([0-9.eE+-]\+\)[[:space:]]*s.*|\1|p' "${log_file}" | head -n1)"
    sampled="$(sed -n 's/.*采样次数(实际):[[:space:]]*\([0-9]\+\).*/\1/p' "${log_file}" | head -n1)"

    line="${robot},${t},${total_ms:-NA},${valid_ms:-NA},${wall_pct:-NA},${valid_calls:-NA},${actual_s:-NA},${sampled:-NA}"
    printf "%s\n" "${line}" | tee -a "${OUT_CSV}"

    t_tag="${t//./p}"
    mv "${log_file}" "${REPO}/scripts/log_sim_rrt_time_${robot}_${t_tag}.log"
  done
done




