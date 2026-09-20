#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${BACON_REPO_ROOT}/exp/collision_rate_obb/"
mkdir -p "$DATA_DIR"/{prbt,fanuc,panda,jaco2,jaco3,go1}
find "$DATA_DIR" -type f -name "*.txt" | while read -r file; do
    > "$file"
    echo "clean: $file"
done

${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/prbt/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/prbt/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/prbt/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_prbt -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/prbt/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/fanuc/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/fanuc/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/fanuc/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_fanuc -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/fanuc/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/panda/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/panda/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/panda/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_panda -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/panda/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/jaco2/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/jaco2/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/jaco2/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco2 -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/jaco2/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/jaco3/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/jaco3/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/jaco3/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_jaco3 -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/jaco3/48.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/8 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/go1/8.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/16 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/go1/16.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/32 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/go1/32.txt
${BACON_REPO_ROOT}/build/joint_collision_rate_go1 -d ${BACON_REPO_ROOT}/env/48 >> ${BACON_REPO_ROOT}/exp/collision_rate_obb/go1/48.txt

