#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${BACON_REPO_ROOT}/exp/collision_id/"
find "$DATA_DIR" -type f -name "*.txt" | while read -r file; do
    > "$file"
    echo "clean: $file"
done

${BACON_REPO_ROOT}/build/sim_cpu_prbt  -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_id/prbt/8.txt
${BACON_REPO_ROOT}/build/sim_cpu_prbt  -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_id/prbt/16.txt
${BACON_REPO_ROOT}/build/sim_cpu_prbt  -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_id/prbt/32.txt
${BACON_REPO_ROOT}/build/sim_cpu_prbt  -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_id/prbt/48.txt
${BACON_REPO_ROOT}/build/sim_cpu_fanuc  -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_id/fanuc/8.txt
${BACON_REPO_ROOT}/build/sim_cpu_fanuc  -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_id/fanuc/16.txt
${BACON_REPO_ROOT}/build/sim_cpu_fanuc  -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_id/fanuc/32.txt
${BACON_REPO_ROOT}/build/sim_cpu_fanuc  -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_id/fanuc/48.txt
${BACON_REPO_ROOT}/build/sim_cpu_panda  -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_id/panda/8.txt
${BACON_REPO_ROOT}/build/sim_cpu_panda  -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_id/panda/16.txt
${BACON_REPO_ROOT}/build/sim_cpu_panda  -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_id/panda/32.txt
${BACON_REPO_ROOT}/build/sim_cpu_panda  -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_id/panda/48.txt
${BACON_REPO_ROOT}/build/sim_cpu_jaco2  -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_id/jaco2/8.txt
${BACON_REPO_ROOT}/build/sim_cpu_jaco2  -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_id/jaco2/16.txt
${BACON_REPO_ROOT}/build/sim_cpu_jaco2  -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_id/jaco2/32.txt
${BACON_REPO_ROOT}/build/sim_cpu_jaco2  -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_id/jaco2/48.txt
${BACON_REPO_ROOT}/build/sim_cpu_jaco3  -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_id/jaco3/8.txt
${BACON_REPO_ROOT}/build/sim_cpu_jaco3  -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_id/jaco3/16.txt
${BACON_REPO_ROOT}/build/sim_cpu_jaco3  -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_id/jaco3/32.txt
${BACON_REPO_ROOT}/build/sim_cpu_jaco3  -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_id/jaco3/48.txt
${BACON_REPO_ROOT}/build/sim_cpu_go1  -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_id/go1/8.txt
${BACON_REPO_ROOT}/build/sim_cpu_go1  -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_id/go1/16.txt
${BACON_REPO_ROOT}/build/sim_cpu_go1  -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_id/go1/32.txt
${BACON_REPO_ROOT}/build/sim_cpu_go1  -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_id/go1/48.txt
