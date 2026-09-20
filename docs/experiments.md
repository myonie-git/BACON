# Experiment entry points

These are code-to-paper connections identified from the supplied BACON paper
and the working tree. They are not a claim that one command regenerates every
published number. The RTL repository is maintained separately.

Build an individual target using `cmake --build build --target NAME -j4`.
Most target families have suffixes `fanuc`, `prbt`, `panda`, `jaco2`, `jaco3`,
and `go1`. The original `mtcs` spelling is preserved in filenames.

| Programs / scripts | Purpose and paper connection |
|---|---|
| `joint_collision_rate_*`, `joint_obb_rate_*`, `count_collision_rate*.py` | Geometry collision statistics related to Fig. 5 |
| `sim_cpu_*`, `openmp_sim_cpu_*`, `sim_gpu_latency_*`, `sim_gpu_throughput_*` | Software timing experiments related to Figs. 11–12; backend caveat below |
| `test_obb_time.sh`, `obb_time.py`, `draw_obb_time.py` | AABB/OBB timing analysis related to Fig. 13 |
| `sim_n_size_prbt` | PRBT group-count exploration related to Fig. 14 |
| `sim_prrt_plan`, `sim_prrt_sample_limit`, `sim_rrt_sample_limit`, `run_*limit.sh` | Planning and fixed-sample timing; related to Fig. 15 |
| `sim_group_dynamic_timer_prbt`, `sim_group_dynamic_timer_go1`, `draw_group_dynamic.sh` | Dynamic-scene exploration related to Fig. 16 |
| `sim_group_timer_prbt_insight2`, `env/insight2` | Scheduling-order exploration related to Fig. 17 |
| `sim_group_timer_*`, `test_strategy_*.py` | Scheduling strategy comparison related to Figs. 18–19 |
| `test_software.py`, `draw_speedup.py` | Software grouping analysis related to Fig. 20 |
| `mtcs_step_*`, `sim_group_timer_*_no_mtcs`, `test_*_no_mtcs.py` | MCTS grouping and kinematic-link grouping comparisons related to Figs. 21–22 |
| `j2_prior.urdf`, `sim_group_timer_jaco2_prior_work` | Prior-work software model associated with Table III |

## Interpretation and known limits

- This repository's `sim_cpu_*` calls its own `CollisionEnv`. The CUDA programs
  implement custom overlap kernels. Although the source uses FCL types and
  geometry helpers, these entries must not be labeled the paper's final
  FCL/MoveIt and cuRobo baselines without identifying the external runners and
  matching experiment configuration. No cuRobo runner was found in this tree.
- The paper describes a one-million-task experiment, 12 CPU threads and GPU
  batches of 4096. Historical programs also contain joint-grid sweeps. Inspect
  each program's sample count, robot, scenes and mode before comparing results.
- `test_latency.py` currently enables CPU entries only;
  `test_strategy_latency.py` enables Jaco-3 entries only. Other choices remain
  commented out. Drivers may skip unavailable binaries and emit missing values.
  They are preserved research scripts, not a completeness-checked paper pipeline.
- `sim_group_timer_*` reports values from the project's timing model. Its
  modeled quantities are distinct from elapsed wall time reported by CPU runs.
  Check the units and invocation mode in each program before plotting comparisons.
- The patched pRRT keeps searching after finding a solution until its external
  termination condition fires. Its fixed-sample measurements are not by
  themselves a reproduction of the paper's Simple/Medium/Hard control-frequency
  comparison. Preserve the patch when using these drivers.
- A pRRT sample counter can exceed a small requested limit because sampling
  and termination checks occur at different points. Inspect the reported
  actual count before treating a run as an exact fixed-sample comparison.
- The three legacy `insight2_*_rate_openmp` programs mutate a shared model
  inside their parallel loops. Multi-threaded correctness has not been
  established; avoid using their multi-threaded output as research evidence.
  They are not the six corrected `openmp_sim_cpu_*` baselines.
- MCTS programs have substantial built-in searches. Old RAM/test-vector
  generators may expect output subdirectories such as `data/ram` to exist.
  They remain available as source, but are not part of the default smoke check.
- Sphere-related scripts are retained, but commented sphere experiments in
  the current collision implementation do not establish an active sphere
  backend. Check the implementation before interpreting those script names.
- CUDA, broad experiment sweeps, hardware results and the published numerical
  claims were not revalidated as part of removing archived results.

Keep regenerated results under ignored output directories. Record the exact
source revision, dependency versions, command, seed, scene and robot selection
alongside any new measurement used in a paper or benchmark.
