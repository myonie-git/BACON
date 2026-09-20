#!/usr/bin/env bash
BACON_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
${BACON_REPO_ROOT}/build/test_aabb_overlap_gen
${BACON_REPO_ROOT}/build/test_aabb_ram_gen
${BACON_REPO_ROOT}/build/test_aabb_tf_gen
${BACON_REPO_ROOT}/build/test_angle_axis_gen
${BACON_REPO_ROOT}/build/test_compute_tf_gen
${BACON_REPO_ROOT}/build/test_obb_overlap_gen
${BACON_REPO_ROOT}/build/test_obb_ram_gen
${BACON_REPO_ROOT}/build/test_obb_tf_gen
${BACON_REPO_ROOT}/build/test_top_gen
${BACON_REPO_ROOT}/build/test_top_fk_gen
${BACON_REPO_ROOT}/build/test_tree_traversal_gen
${BACON_REPO_ROOT}/build/test_top_tf_ram_gen
${BACON_REPO_ROOT}/test/test_bvh_tree_to_ram.cpp
