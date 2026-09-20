#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${BACON_REPO_ROOT}/exp/collision_rate_sphere/"
mkdir -p "$DATA_DIR"/{prbt,fanuc,panda,jaco2,jaco3,go1}
find "$DATA_DIR" -type f -name "*.txt" | while read -r file; do
    > "$file"
    echo "clean: $file"
done

${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/prbt/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/prbt/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/prbt/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/prbt/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/fanuc/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/fanuc/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/fanuc/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/fanuc/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/panda/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/panda/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/panda/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/panda/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/jaco2/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/jaco2/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/jaco2/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/jaco2/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/jaco3/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/jaco3/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/jaco3/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/jaco3/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/go1/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/go1/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/go1/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_sphere/go1/48.txt

