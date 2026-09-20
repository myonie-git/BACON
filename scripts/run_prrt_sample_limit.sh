#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# 统一 seed 下，批量跑 sim_prrt_sample_limit 的六个机械臂与三组 samples。

set -euo pipefail

REPO="${REPO:-${BACON_REPO_ROOT}}"
BIN="${BIN:-${REPO}/build/sim_prrt_sample_limit}"
ENV_DIR="${ENV_DIR:-${REPO}/env/48}"
ENV_ID="${ENV_ID:-2}"
THREADS="${THREADS:-4}"
SEED="${SEED:-123456}"
OUT_CSV="${OUT_CSV:-${REPO}/scripts/prrt_sample_limit_results.csv}"

if [[ ! -x "${BIN}" ]]; then
  echo "可执行文件不存在或不可执行: ${BIN}" >&2
  echo "请先在工程根目录编译，或通过 BIN 环境变量指定路径。" >&2
  exit 1
fi

robots=(fanuc go1 jaco2 jaco3 panda prbt)
samples=(200 500 1000 2500 5000 8000 10000)

mkdir -p "$(dirname "${OUT_CSV}")"
printf "robot,samples,total_ms,valid_wall_ms,wall_pct,valid_calls\n" | tee "${OUT_CSV}"

for robot in "${robots[@]}"; do
  for n in "${samples[@]}"; do
    log_file="$(mktemp)"
    if ! "${BIN}" \
      --robot "${robot}" \
      --threads "${THREADS}" \
      --samples "${n}" \
      --seed "${SEED}" \
      --env-dir "${ENV_DIR}" \
      --env-id "${ENV_ID}" \
      --repo "${REPO}" \
      >"${log_file}" 2>&1; then
      echo "运行失败: robot=${robot}, samples=${n}" >&2
      cat "${log_file}" >&2
      rm -f "${log_file}"
      exit 1
    fi

    total_ms="$(sed -n 's/.*总耗时 (wall):[[:space:]]*\([0-9.eE+-]\+\).*/\1/p' "${log_file}" | head -n1)"
    valid_ms="$(sed -n 's/.*状态有效性检查耗时(墙钟累计):[[:space:]]*\([0-9.eE+-]\+\).*/\1/p' "${log_file}" | head -n1)"
    wall_pct="$(sed -n 's/.*状态有效性检查耗时(墙钟累计):.*占墙钟约[[:space:]]*\([0-9.eE+-]\+\)%.*/\1/p' "${log_file}" | head -n1)"
    valid_calls="$(sed -n 's/.*状态有效性检查调用次数:[[:space:]]*\([0-9]\+\).*/\1/p' "${log_file}" | head -n1)"

    line="${robot},${n},${total_ms:-NA},${valid_ms:-NA},${wall_pct:-NA},${valid_calls:-NA}"
    printf "%s\n" "${line}" | tee -a "${OUT_CSV}"

    mv "${log_file}" "${REPO}/scripts/log_sim_prrt_${robot}_${n}.log"
  done
done

