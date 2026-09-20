# Source snapshot policy

The publication candidate is a new snapshot of the research working tree,
including selected staged and untracked source files. It does not inherit the
research repository's Git history. The original research checkout, local
commits, staging state and raw data are retained separately.

## Included

- Core C++ sources and headers, experiment sources, CUDA sources, Python and
  shell drivers, and the no-MCTS, dynamic-scene, OpenMP and planning additions.
- URDF robot descriptions, model meshes and required vendored geometry helpers.
- Small obstacle inputs in `env/8`, `env/16`, `env/32`, `env/48`, `env/4`,
  `env/insight2`, `env/48-bak`, and `env/box`, plus root `boxes*.txt`.
  `env/48-bak` is retained because current MCTS programs use it as their default
  input; its name does not make it an experiment result.
- OMPL's exact revision and a patch including the two previously untracked
  demos. OMPL's downloaded source is an optional local dependency.
- Build instructions, the paper citation, licensing status and a smoke check.

## Excluded

- All historical `exp/` outputs, including per-query collision traces, OBB rate
  logs, OBB timing data and prior experiment tables.
- Generated CSV tables, plotted PDFs, logs, MoveIt run outputs, raw MCTS
  output, RAM/test-vector dumps, `data/`, and `data_bak/`.
- Build products, caches, editor settings, profiling output, compiled Python
  files, scratch `tmp*.cpp` files and the unused `mtcs_step_panda_bak.cpp`.
- Generated catkin CMake package files copied into the Franka model directory.
- The unused Go2 resource directory and unreferenced scene backup `env/oops`.
- The old `.git` history, the nested OMPL Git metadata and the paper PDF.

Robot meshes account for most of the retained bytes. They are model inputs,
not benchmark results. No result CSV or figure is included as a sample. To add
a curated result later, document the command, revision, parameters and source
data separately from this source-only snapshot.

The ignore rules target output paths and common generated file locations.
They do not ignore all `.txt` files or the `env/` directory, so necessary small
inputs remain versioned. `.gitignore` cannot remove large files from an old
commit; that is why this snapshot starts a separate history.
