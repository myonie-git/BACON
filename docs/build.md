# Build and usage

## Build and smoke check

The existing dependency stack uses ROS 1 libraries. The checked environment is
Ubuntu 20.04 / ROS Noetic, CMake 3.24, GCC 13, Eigen 3.3.7, Boost 1.71, FCL 0.7,
Assimp 5.0, TinyXML2 6.2, and OctoMap 1.9.8. These are validation versions,
not a claim that all other versions are unsupported.

Install a C++17 compiler, CMake >= 3.16, pkg-config, Boost system/filesystem,
Eigen, TinyXML2, FCL, Assimp, OctoMap, Qhull, console_bridge, and the ROS
`random_numbers`, `resource_retriever`, `shape_msgs`, and `visualization_msgs`
packages. FCL and TinyXML2 must be discoverable by pkg-config. If FCL is installed
in a separate prefix, expose its pkg-config directory through `PKG_CONFIG_PATH`.
No developer-specific catkin workspace is required by the source files.

From the repository root, with those dependencies installed:

```bash
source /opt/ros/noetic/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

Use a consistent compiler/dependency environment. An active Conda environment
may cause CMake to select a different Boost or runtime library.

The default build compiles `cdu_impl`, `cdu_cimpl`, and `bacon_smoke`; it does
not launch an experiment sweep. The smoke check loads all six robot models,
checks basic AABB/OBB overlaps, and verifies copied-model joint poses. It runs
without the original research directory or any archived result files.

Mesh paths are resolved relative to each URDF. Program defaults for model and
scene paths are set from the source directory at configure time. Reconfigure
after moving the source tree. The smoke check uses collision geometry; PRBT's
optional visual meshes refer to the external `moveit_resources_prbt_support`
ROS package.

## Run an experiment

Experiment targets are built explicitly. For example:

```bash
cmake --build build -j4 --target sim_cpu_prbt openmp_sim_cpu_prbt sim_group_timer_prbt
repo="$PWD"
mkdir -p outputs/prbt
cd outputs/prbt
"$repo/build/sim_cpu_prbt" -d "$repo/env/8"
"$repo/build/openmp_sim_cpu_prbt" -d "$repo/env/8" -t 12
"$repo/build/sim_group_timer_prbt" -d "$repo/env/8" --g
```

The experiment programs can be much slower than the smoke check. The historical
batch scripts retain their original selections and sample counts; inspect them
before starting a full run. Python analysis dependencies are listed in
`requirements.txt`. Plotting scripts expect locally generated input results.
Run scripts from a directory under `outputs/` to keep relative CSV, PDF and log
outputs together. Some older drivers explicitly write under `exp/`, `data/` or
`scripts/`; those generated outputs are also ignored by Git.

See [experiment guide](experiments.md) for target families, paper
connections, and the limits of this software snapshot.

## Optional dependencies

- OpenMP is enabled by default. Disable it with `-DBACON_ENABLE_OPENMP=OFF`;
  the OpenMP-specific targets will then be unavailable.
- CUDA targets are enabled with `-DBACON_ENABLE_CUDA=ON`. They need a CUDA
  compiler supporting C++17 and a compatible host compiler. CUDA execution has
  not been validated in this release preparation. These historical kernels are
  custom CUDA baselines; they are not a cuRobo integration.
- RRT/pRRT uses an exact OMPL source revision plus the original local patch:

  ```bash
  python3 scripts/setup_ompl.py
  cmake -S . -B build -DBACON_ENABLE_OMPL=ON
  cmake --build build -j4 --target sim_rrt_sample_limit sim_prrt_sample_limit sim_prrt_plan
  ```

  This OMPL revision also requires yaml-cpp and Boost serialization and
  program_options. VAMP, Python bindings, OMPL demos and upstream tests are
  disabled in this integration.

  OMPL is downloaded only by the explicit setup command. Its commit, archive
  hash and patch hash are recorded in `third_party/ompl.lock.json`. The downloaded
  tree is ignored by Git. `python3 scripts/setup_ompl.py --check` verifies the
  prepared version and patched files without network access. An offline archive
  can be supplied using `--archive /path/to/ompl.tar.gz` with the same checksum.
