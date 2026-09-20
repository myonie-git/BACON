#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${BACON_REPO_ROOT}/exp/collision_rate/"
mkdir -p "$DATA_DIR"/{prbt,fanuc,panda,jaco2,jaco3,go1}
find "$DATA_DIR" -type f -name "*.txt" | while read -r file; do
    > "$file"
    echo "clean: $file"
done

${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate/prbt/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate/prbt/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate/prbt/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate/prbt/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate/fanuc/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate/fanuc/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate/fanuc/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate/fanuc/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate/panda/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate/panda/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate/panda/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate/panda/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate/jaco2/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate/jaco2/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate/jaco2/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate/jaco2/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate/jaco3/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate/jaco3/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate/jaco3/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate/jaco3/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate/go1/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate/go1/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate/go1/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate/go1/48.txt

