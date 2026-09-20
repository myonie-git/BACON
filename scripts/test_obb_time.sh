#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${BACON_REPO_ROOT}/exp/obb_time/"
find "$DATA_DIR" -type f -name "*.txt" | while read -r file; do
    > "$file"
    echo "clean: $file"
done

${BACON_REPO_ROOT}/build/sim_group_timer_prbt --g -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/obb_time/prbt/8.txt
${BACON_REPO_ROOT}/build/sim_group_timer_prbt --g -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/obb_time/prbt/16.txt
${BACON_REPO_ROOT}/build/sim_group_timer_prbt --g -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/obb_time/prbt/32.txt
${BACON_REPO_ROOT}/build/sim_group_timer_prbt --g -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/obb_time/prbt/48.txt
${BACON_REPO_ROOT}/build/sim_group_timer_fanuc --g -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/obb_time/fanuc/8.txt
${BACON_REPO_ROOT}/build/sim_group_timer_fanuc --g -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/obb_time/fanuc/16.txt
${BACON_REPO_ROOT}/build/sim_group_timer_fanuc --g -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/obb_time/fanuc/32.txt
${BACON_REPO_ROOT}/build/sim_group_timer_fanuc --g -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/obb_time/fanuc/48.txt
${BACON_REPO_ROOT}/build/sim_group_timer_panda --g -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/obb_time/panda/8.txt
${BACON_REPO_ROOT}/build/sim_group_timer_panda --g -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/obb_time/panda/16.txt
${BACON_REPO_ROOT}/build/sim_group_timer_panda --g -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/obb_time/panda/32.txt
${BACON_REPO_ROOT}/build/sim_group_timer_panda --g -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/obb_time/panda/48.txt
${BACON_REPO_ROOT}/build/sim_group_timer_jaco2 --g -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/obb_time/jaco2/8.txt
${BACON_REPO_ROOT}/build/sim_group_timer_jaco2 --g -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/obb_time/jaco2/16.txt
${BACON_REPO_ROOT}/build/sim_group_timer_jaco2 --g -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/obb_time/jaco2/32.txt
${BACON_REPO_ROOT}/build/sim_group_timer_jaco2 --g -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/obb_time/jaco2/48.txt
${BACON_REPO_ROOT}/build/sim_group_timer_jaco3 --g -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/obb_time/jaco3/8.txt
${BACON_REPO_ROOT}/build/sim_group_timer_jaco3 --g -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/obb_time/jaco3/16.txt
${BACON_REPO_ROOT}/build/sim_group_timer_jaco3 --g -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/obb_time/jaco3/32.txt
${BACON_REPO_ROOT}/build/sim_group_timer_jaco3 --g -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/obb_time/jaco3/48.txt
${BACON_REPO_ROOT}/build/sim_group_timer_go1 --g -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/obb_time/go1/8.txt
${BACON_REPO_ROOT}/build/sim_group_timer_go1 --g -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/obb_time/go1/16.txt
${BACON_REPO_ROOT}/build/sim_group_timer_go1 --g -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/obb_time/go1/32.txt
${BACON_REPO_ROOT}/build/sim_group_timer_go1 --g -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/obb_time/go1/48.txt

python3 obb_time.py
