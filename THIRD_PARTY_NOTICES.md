# Third-party components and assets

This inventory records the notices and metadata available in the research
checkout. It is not a replacement for the respective licenses. Existing source
headers and attribution files have been retained.

| Component | Recorded source / evidence | License and notes |
|---|---|---|
| `geometric_shapes/` | [MoveIt geometric_shapes](https://github.com/moveit/geometric_shapes); package metadata and source headers | BSD notices are retained in individual source files; the vendored tree has local changes. |
| OMPL | Revision in `third_party/ompl.lock.json`; [OMPL](https://github.com/ompl/ompl) | BSD terms in `third_party/OMPL-LICENSE` and upstream source headers. Local changes are in `third_party/ompl-local.patch`. |
| Fanuc resources | `resources/fanuc_description/README.md` and `LICENSE`; adapted MoveIt / ROS-Industrial resources | Retain the supplied `resources/fanuc_description/LICENSE`. |
| Franka / Panda resources | [franka_ros 0.10.1, commit 30e598a](https://github.com/frankaemika/franka_ros/tree/30e598aa6fb703cc80a203481e6427f397337b4c/franka_description) | Apache 2.0. Upstream `LICENSE` and `NOTICE` are included under `resources/franka_description/`. All 31 retained original resource files match upstream Git blob hashes. Two generated catkin CMake files were excluded. |
| Kinova resources | `resources/kinova_description/meta-information.json` links [commit 0952157](https://github.com/Kinovarobotics/kinova-ros/tree/09521579f49ffc8af88aafd749450ae9ed718ec9/kinova_description) | Upstream BSD license is included at `resources/kinova_description/LICENSE`. The snapshot has modified DAE files, generated standalone URDFs and package metadata; see below. |
| Go1 resources | [Unitree unitree_ros, commit ccfc6fd](https://github.com/unitreerobotics/unitree_ros/tree/ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee/robots/go1_description) | Upstream BSD-3-Clause license is included at `resources/go1_description/LICENSE`. All 21 original bundled files match upstream Git blob hashes. The historical `TODO` field in package.xml is retained; the recovered upstream repository license supplies the missing notice. |
| PRBT description | Generated from the locally retained `moveit_resources_prbt_support` package, version 0.8.3; its upstream commit was not recorded | Apache 2.0 terms recovered from that package are included at `third_party/PRBT-LICENSE`. The URDF retains the Pilz copyright notice. Collision shapes are in the URDF; optional external visual meshes are not bundled. |

Kinova comparison: 28 original resource files match the recorded upstream
revision. The package metadata and seven meshes differ: `arm_half_1.dae`,
`arm_half_2.dae`, `arm_mico.dae`, `forearm_mico.dae`, `hand_2finger.dae`,
`wrist_spherical_1.dae`, and `wrist_spherical_2.dae`. The metadata JSON and
generated standalone URDFs are additional files. These working-tree assets
were retained without changing their geometry during publication preparation;
they should not be described as an unmodified upstream package.

Some BACON geometry headers refer to FCL. FCL and other linked dependencies are
provided by the build environment and keep their upstream licenses. The
top-level MIT license covers the original BACON code; it does not relicense
third-party components or derived upstream portions. Retain their copyright
headers and notices when redistributing the source.
