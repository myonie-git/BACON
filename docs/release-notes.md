# Publication preparation — 2026-09-20

## Provenance and selection

This is a new source snapshot of the research `xyc` working tree based on
commit `1d0fafbdb2a66874a7ccbb1d26f48a3f9d83d60f`, including selected staged
and untracked additions. It does not include the old repository history or
archived experiment outputs. The original checkout remains the research archive.

The omitted `exp/` files alone account for about 10.81 GiB of tracked content
at that research commit. Additional ignored logs and run results are also left
out. No scientific results were edited, replaced, or regenerated for this release.
See [source-selection.md](source-selection.md) for the included input data.

## Changes made to the publication copy

- Replaced developer-specific paths with source-relative CMake defaults,
  script-relative paths, and URDF-relative mesh resolution. Missing collision
  meshes now cause an explicit error.
- Consolidated shared C++ sources into `cdu_impl`, made experiment targets
  explicit build requests, and made CUDA and OMPL optional. FCL and TinyXML2
  include/link settings come from pkg-config; no absolute library paths are
  hardcoded into CMake.
- Added a small default executable and CTest entry covering the six robot
  models, basic overlap behavior and copied-model joint poses.
- Corrected the six PRBT joint names in `openmp_sim_cpu_prbt` to match its URDF.
  Updated the six `openmp_sim_cpu_*` programs to construct collision objects
  using each copied model's link poses. The old shared `CollisionGeom` pointer
  otherwise still referenced the original model's links. The general model
  copy behavior and the separate legacy insight2 programs were not redesigned.
- Preserved the exact local OMPL changes as a patch against revision
  `8f60adae80c8138880650df2ee527f7b11a94004`, including the two new demos.
  Added a checksum-verified setup command instead of embedding the dependency
  checkout and its Git history.
- Added focused ignore rules, documentation, citation metadata and recovered
  third-party license/notice files. The maintainer selected MIT for the original
  BACON code; third-party terms remain separate.

The pose/joint corrections change the affected programs' behavior. Measurements
from this snapshot should not be assumed numerically identical to older runs.

## Validation performed

- Release build of the default targets succeeded using the dependencies listed
  in the README. CTest passed the six-model smoke check from outside the source
  directory. A second copy exported from the publication Git file list also
  built and passed CTest, without ignored dependencies or experiment outputs.
  The model collision-object counts were 7, 17, 53, 11, 13, and 29
  for Fanuc, PRBT, Panda, Jaco-2, Jaco-3, and Go1 respectively.
- Built `sim_cpu_prbt`, `openmp_sim_cpu_prbt`, `sim_group_timer_prbt`,
  `sim_group_timer_prbt_no_mtcs`, `sim_group_dynamic_timer_prbt`, and
  `mtcs_step_prbt`. Ran the first four against one retained scene, with two
  threads for the OpenMP case; all exited successfully with numeric output.
- Downloaded the pinned OMPL archive, checked its SHA-256, applied the patch,
  and passed the setup check. Built all three RRT/pRRT entry points. Small PRBT
  runs of `sim_rrt_sample_limit` and `sim_prrt_sample_limit` exited successfully;
  `sim_prrt_plan --help` also succeeded. These checks establish integration,
  not planning quality or benchmark reproduction.
- Parsed all 27 Python files and checked all 17 shell scripts with `bash -n`.
  Parsed all 11 top-level / CUDA URDFs and checked their 181 local mesh
  references. The seven PRBT visual package references remain external.
- Checked the publication file list for archived output paths, CSV/PDF/log
  artifacts and files over 100 MiB. No such files are included. Necessary scene
  inputs remain versioned. A limited credential-pattern scan of small text
  files found no matches; it is not a complete security audit.

The build used the machine's installed ROS/FCL dependencies; dependency
installation in a fresh operating-system image was not tested. CUDA execution,
upstream geometric_shapes tests, full experiment sweeps, performance claims and
hardware behavior were not validated. This repository is the software portion;
see [experiments.md](experiments.md) for the scope of the paper connections.
